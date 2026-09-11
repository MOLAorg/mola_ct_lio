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

  /** Relative motion over this segment as reported by an independent
   * odometry source, in the begin knot's frame, and whether there is one.
   *
   * A legged platform's kinematic-inertial estimator does not care what the
   * scene looks like, which is exactly the complement of a LiDAR in a
   * confined or self-similar space.
   */
  SE3 odometryDelta;
  bool hasOdometry = false;

  /** How much of the segment the points actually span, as a fraction.
   *
   * A continuous-time segment can only fit an interpolation if its points
   * carry distinct times. When a provider hands over clouds it has already
   * motion-compensated, every point of a scan shares one instant, and a
   * segment holding a single scan collapses to one alpha: its end knot then
   * receives no LiDAR information whatsoever. Nothing about that fails
   * loudly, so it is worth measuring.
   */
  double alphaSpread = 0;
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

/** How the LiDAR block's weight is reconciled with its own residuals.
 *
 * GICP's information matrix is built from the local scatter of points, so its
 * absolute scale describes a surface's shape rather than the sensor's noise,
 * and it moves with the scene. The inertial factors, by contrast, are
 * whitened by a genuine noise covariance. Balancing the two with a hand-tuned
 * constant therefore cannot work across datasets: the constant is cancelling
 * something that is not constant.
 *
 * The residuals themselves carry the missing scale. A block whose reduced
 * chi-square is far from one is claiming a precision it does not have, or
 * hiding one it does, and rescaling it by that ratio makes its implied noise
 * match what is actually observed.
 */
enum class LidarBalance
{
  /// Take the information matrices at face value.
  None,

  /// Scale the block down when its residuals exceed what it claims, and
  /// never up. This is the conventional Birge ratio, and it is inert when a
  /// block is under-confident rather than over-confident.
  DownOnly,

  /// Scale in both directions, so an under-confident block is also corrected.
  TwoSided,
};

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

    /// Largest velocity change any single knot may take in one iteration.
    ///
    /// The translation limit above does not cover this. A window can propose
    /// a step whose position block is unremarkable and whose velocity block
    /// is enormous, and that step passes: measured on a diverging run, the
    /// velocity state goes from 0.05 to 38 m/s in one window while the
    /// translation limit never fires. Velocity is the state an inconsistency
    /// between the geometry and the inertial term ends up in, so it needs its
    /// own bound. [m/s]
    double maxStepVelocity = 2.0;

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
    /// How far a knot's velocity may stray from zero in absolute terms.
    /// Loose enough that real motion never feels it; it exists so that a
    /// velocity no platform can reach is not silently acceptable. Zero
    /// disables. [m/s]
    double velocityPriorSigma = 5.0;

    double biasPriorSigmaAcc = 0.3;
    double biasPriorSigmaGyro = 0.02;

    /// How the LiDAR block's weight is reconciled against its own residuals.
    /// See LidarBalance.
    LidarBalance lidarBalance = LidarBalance::TwoSided;

    /// The furthest the balance may scale the LiDAR block in either
    /// direction. A window with very few correspondences can produce a wild
    /// ratio, and this is what stops one from being acted on.
    double lidarBalanceMaxScale = 1000.0;

    /// Degrees of freedom the LiDAR block must have before its reduced
    /// chi-square is treated as a scale worth acting on.
    ///
    /// The estimate's own relative error is about sqrt(2/dof), so at 200 it
    /// is good to some ten percent and below a few dozen it is nearly
    /// meaningless. Acting on a meaningless ratio is not merely useless here:
    /// a starved window is exactly where the block is closest to rank
    /// deficient, and amplifying a handful of correspondences by up to the
    /// cap is how a brief loss of returns turns into a lasting one.
    double lidarBalanceMinDof = 200.0;

    /// Uncertainty of an external odometry's relative motion over one
    /// segment. Zero on either disables that half, which is the default: the
    /// factor is measured and useful, but a second pose source fused while
    /// the LiDAR-inertial core still has unexplained failures only makes
    /// those harder to read. [m] and [rad] per segment
    double odometrySigmaLin = 0.0;
    double odometrySigmaAng = 0.0;

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

    /// The LiDAR block's own chi-square and its degrees of freedom, and the
    /// factor the balance applied to it. A scale far from one says the
    /// information matrices were not describing the sensor's noise.
    double lidarChi2 = 0;
    double lidarDof = 0;
    double lidarScale = 1.0;
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

  /// The LiDAR contributions alone, kept apart so that they can be rescaled
  /// as a block once their residuals are known, before joining the rest.
  WindowSystem lidarSystem_;

  /// Correspondences of each segment, kept between re-matches.
  std::vector<std::vector<PointCorrespondence>> correspondences_;

  void addTwistContinuity(const std::vector<Knot> & knots);

  /** Adds the relative-motion terms of whichever segments carry an external
   * odometry measurement.
   */
  void addOdometry(const std::vector<Knot> & knots, const std::vector<Segment> & segments);

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
