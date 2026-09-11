/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   WindowOptimizer.h
 * @brief  Joint optimization of the control knots inside the window
 */
#pragma once

#include <mola_ct_lio/CtNormalEquations.h>
#include <mola_ct_lio/WindowSystem.h>

#include <functional>
#include <vector>

namespace mola::ct
{
/** One control knot of the trajectory. */
struct Knot
{
  double t = 0;
  KnotState state;

  /** Set once the knot has taken part in a marginalization. From then on its
   * Jacobians must be evaluated at `linearizationPoint`, not at the current
   * estimate, or the prior and the new data stop describing the same
   * quantity and the estimator quietly gains information it never had.
   *
   * The point is captured the first time the knot is frozen and never moved
   * again while it lives in the window: re-capturing it on every slide would
   * mean each prior inherits Jacobians taken somewhere else, which is the
   * very inconsistency the fixed point exists to avoid.
   */
  bool linearized = false;
  KnotState linearizationPoint;

  /** Where this knot stood when the current prior's gradient was evaluated.
   *
   * Distinct from `linearizationPoint`: that one fixes *where the Jacobians
   * are taken*, this one fixes *what the prior's gradient is a gradient at*.
   * They coincide the first time a knot is marginalized and part company on
   * every slide after it, so re-centering the prior on the wrong one leaves a
   * residual gradient that grows with the window's own corrections.
   */
  KnotState priorAnchor;

  [[nodiscard]] const KnotState & jacobianState() const
  {
    return linearized ? linearizationPoint : state;
  }
};

/** The data of the interval between two consecutive knots.
 *
 * `segments[k]` spans `knots[k]` to `knots[k+1]`.
 */
struct Segment
{
  /// Source points, as raw sensor readings each carrying its own normalized
  /// time within the segment. See SegmentPoint.
  std::vector<SegmentPoint> points;

  PreintegratedImu imu;
  bool hasImu = false;
};

/** Supplies the correspondences of one segment against the map.
 *
 * Called once per segment per iteration, with the segment placed at the
 * current estimate. Implementations deskew the points with `segment`, query
 * the map, and append one PointCorrespondence per accepted pairing; the
 * `information` they set is what selects point-to-point, point-to-plane or
 * GICP.
 *
 * The order in which correspondences are appended fixes the floating-point
 * summation order of the assembly, so it must be reproducible for the whole
 * estimator to be.
 */
using MatchFunction = std::function<void(
  std::size_t segmentIndex, const CtSegment & segment, const std::vector<SegmentPoint> & points,
  std::vector<PointCorrespondence> & out)>;

/** Joint Gauss-Newton optimization of every knot in the window. */
class WindowOptimizer
{
public:
  struct Params
  {
    int maxIterations = 25;

    /// Convergence is declared when the increment's translation norm falls
    /// below this. [m]
    double convergenceThreshold = 1e-3;

    /// Levenberg damping, as a fraction of the diagonal. Zero is plain
    /// Gauss-Newton, which is what upstream Traj-LO uses.
    double lambda = 0.0;

    /// Largest translation any single knot may be moved by one iteration. A
    /// step asking for more is scaled down as a whole, keeping its direction.
    /// Gauss-Newton has no trust region of its own, so without this a single
    /// badly conditioned iteration can throw the window far enough that the
    /// next matching finds nothing and the two feed each other. [m]
    double maxStepTranslation = 1.0;

    RobustKernel kernel = RobustKernel::Cauchy;
    double kernelScale = 0.5;

    /// How often the correspondences are searched again, in iterations. One
    /// re-matches every iteration, as upstream Traj-LO does with its cheap
    /// voxel lookup; a larger value amortizes a costlier matcher over several
    /// inner iterations that only move the trajectory. It trades matching cost
    /// against how stale the pairings are allowed to get, so it is worth a
    /// sweep rather than a guess.
    int rematchEvery = 1;

    bool useImu = true;
    Vec3 gravity{0.0, 0.0, -9.81};
    double biasSigmaAcc = 1e-3;
    double biasSigmaGyro = 1e-4;

    /// How far the bias states are allowed to stray from zero in absolute
    /// terms. See assembleBiasPriorBlock(): the random walk alone leaves them
    /// unbounded. Loose enough that a genuine sensor bias never feels it.
    /// Zero disables. [m/s^2] and [rad/s]
    double biasPriorSigmaAcc = 0.3;
    double biasPriorSigmaGyro = 0.02;

    /// Weight of the twist-continuity term used when the IMU is absent. It is
    /// what keeps a LiDAR-only window from drifting in an unobservable
    /// direction, and it plays no part once IMU factors are present.
    double twistContinuityWeight = 2.0;
  };

  WindowOptimizer() = default;
  explicit WindowOptimizer(const Params & p) : params(p) {}

  Params params;

  struct Result
  {
    int iterations = 0;
    bool converged = false;
    double chi2 = 0;
    double errorSum = 0;
    std::size_t inliers = 0;

    /// Whether any iteration asked for a longer step than the trust region
    /// allows. A window that reports this was not simply refining.
    bool stepWasLimited = false;

    /// How much information each source puts on the knots' *position*, summed
    /// over the window's diagonal. Restricting the comparison to one block
    /// makes it apples-to-apples: a trace over the whole state would add
    /// m^-2 to rad^-2 to (m/s)^-2 and mean nothing. These are what decide
    /// whether the LiDAR can still correct the inertial prediction, or is
    /// merely along for the ride. [m^-2]
    double lidarPositionInfo = 0;
    double imuPositionInfo = 0;
    double priorPositionInfo = 0;
  };

  /** Runs the optimization in place.
   *
   * @param knots     The window's control knots, updated in place.
   * @param segments  One per consecutive knot pair, so `knots.size() - 1` of them.
   * @param match     Correspondence supplier, see MatchFunction.
   * @param prior     Marginalization prior over the leading knots, or an
   *                  invalid one on the first windows.
   */
  Result optimize(
    std::vector<Knot> & knots, const std::vector<Segment> & segments, const MatchFunction & match,
    const MarginalizationPrior & prior);

  /** The dimension of one knot's state under the current parameters. */
  [[nodiscard]] int knotDim() const { return params.useImu ? kKnotDim : 6; }

  /** The system left by the last optimize() call, which is what a subsequent
   * marginalization has to be taken from.
   */
  [[nodiscard]] const WindowSystem & lastSystem() const { return system_; }

private:
  WindowSystem system_;

  /// Correspondences of each segment, kept between re-matches.
  std::vector<std::vector<PointCorrespondence>> correspondences_;

  void addTwistContinuity(const std::vector<Knot> & knots);

  /** Builds the normal equations of the whole window at the current states.
   *
   * @param rematch  Whether to search the correspondences again, or reuse the
   *                 ones cached by the last call that did.
   */
  void assemble(
    const std::vector<Knot> & knots, const std::vector<Segment> & segments,
    const MatchFunction & match, const MarginalizationPrior & prior, bool rematch, Result & result);
};

/** The deviation of a knot from a reference state, in the increment convention
 * of incKnot(). Used to re-center a marginalization prior on where the states
 * have since moved to.
 */
[[nodiscard]] Vec15 knotDeviation(const KnotState & current, const KnotState & reference);

}  // namespace mola::ct
