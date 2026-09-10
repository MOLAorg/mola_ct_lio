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
   */
  bool linearized = false;
  KnotState linearizationPoint;

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
  /// Source points, in the body frame of the segment's begin knot, each with
  /// its normalized time within the segment.
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

    RobustKernel kernel = RobustKernel::Cauchy;
    double kernelScale = 0.5;

    bool useImu = true;
    Vec3 gravity{0.0, 0.0, -9.81};
    double biasSigmaAcc = 1e-3;
    double biasSigmaGyro = 1e-4;

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
  mutable std::vector<PointCorrespondence> correspondences_;

  void addTwistContinuity(const std::vector<Knot> & knots);
};

/** The deviation of a knot from a reference state, in the increment convention
 * of incKnot(). Used to re-center a marginalization prior on where the states
 * have since moved to.
 */
[[nodiscard]] Vec15 knotDeviation(const KnotState & current, const KnotState & reference);

}  // namespace mola::ct
