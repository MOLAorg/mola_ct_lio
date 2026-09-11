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
 * @file   WindowOptimizer.cpp
 * @brief  Joint optimization of the control knots inside the window
 */
#include <mola_ct_lio/WindowOptimizer.h>

#include <algorithm>
#include <cmath>

namespace mola::ct
{
namespace
{
/** The decoupled twist between two knots, which is the quantity the
 * continuity term keeps from changing abruptly.
 */
Vec6 twistBetween(const SE3 & Ta, const SE3 & Tb) { return se3LogDecoupled(Ta.inverse() * Tb); }

/** Residual of the twist-continuity term over three consecutive knots. */
Vec6 twistContinuityResidual(const SE3 & T0, const SE3 & T1, const SE3 & T2)
{
  return twistBetween(T1, T2) - twistBetween(T0, T1);
}
}  // namespace

Vec15 knotDeviation(const KnotState & current, const KnotState & reference)
{
  Vec15 d = Vec15::Zero();
  d.segment<3>(kIdxPosition) = current.T.t - reference.T.t;
  d.segment<3>(kIdxRotation) = so3Log(reference.T.R.transpose() * current.T.R);
  d.segment<3>(kIdxVelocity) = current.v - reference.v;
  d.segment<3>(kIdxBiasAcc) = current.biasAcc - reference.biasAcc;
  d.segment<3>(kIdxBiasGyro) = current.biasGyro - reference.biasGyro;
  return d;
}

/** Adds the term that keeps the twist from changing abruptly between
 * consecutive segments, which is what stands in for the IMU when there is
 * none.
 *
 * Its Jacobian is taken numerically. Unlike the LiDAR residual, this one is
 * evaluated once per knot triplet per iteration rather than once per point,
 * so the cost is irrelevant beside the matching, and a closed form here would
 * buy nothing but a chance to get it wrong.
 */
void WindowOptimizer::addTwistContinuity(const std::vector<Knot> & knots)
{
  const double weight = params.twistContinuityWeight;
  if (weight <= 0 || knots.size() < 3) {
    return;
  }

  constexpr double kEps = 1e-7;

  for (std::size_t k = 0; k + 2 < knots.size(); k++) {
    const SE3 & T0 = knots[k].state.T;
    const SE3 & T1 = knots[k + 1].state.T;
    const SE3 & T2 = knots[k + 2].state.T;

    const Vec6 r = twistContinuityResidual(T0, T1, T2);

    Eigen::Matrix<double, 6, 18> J;
    for (int i = 0; i < 18; i++) {
      Vec6 inc = Vec6::Zero();
      inc[i % 6] = kEps;

      SE3 p0 = T0;
      SE3 p1 = T1;
      SE3 p2 = T2;
      SE3 * target = i < 6 ? &p0 : (i < 12 ? &p1 : &p2);
      incPose(*target, inc);

      inc[i % 6] = -kEps;
      SE3 m0 = T0;
      SE3 m1 = T1;
      SE3 m2 = T2;
      SE3 * targetMinus = i < 6 ? &m0 : (i < 12 ? &m1 : &m2);
      incPose(*targetMinus, inc);

      J.col(i) =
        (twistContinuityResidual(p0, p1, p2) - twistContinuityResidual(m0, m1, m2)) / (2.0 * kEps);
    }

    const Eigen::Matrix<double, 18, 18> H = weight * J.transpose() * J;
    const Eigen::Matrix<double, 18, 1> g = -weight * J.transpose() * r;

    system_.addPoseTripletBlock(static_cast<int>(k), H, g);
  }
}

/** Residual of one segment's motion against an external odometry's report of
 * it, in the increment convention the rest of the estimator uses: the
 * translation compared in the world frame, the rotation on the right.
 */
namespace
{
Vec6 odometryResidual(const SE3 & Ti, const SE3 & Tj, const SE3 & measured)
{
  Vec6 r;
  r.head<3>() = (Tj.t - Ti.t) - Ti.R * measured.t;
  r.tail<3>() = so3Log(measured.R.transpose() * (Ti.R.transpose() * Tj.R));
  return r;
}
}  // namespace

/** Adds the relative-motion term of every segment that carries an external
 * odometry measurement.
 *
 * Its Jacobian is numerical, for the same reason the twist-continuity term's
 * is: one evaluation per segment per iteration is nothing beside the
 * matching, and a closed form here would buy only a chance to get it wrong.
 */
void WindowOptimizer::addOdometry(
  const std::vector<Knot> & knots, const std::vector<Segment> & segments)
{
  const double wLin =
    params.odometrySigmaLin > 0 ? 1.0 / (params.odometrySigmaLin * params.odometrySigmaLin) : 0.0;
  const double wAng =
    params.odometrySigmaAng > 0 ? 1.0 / (params.odometrySigmaAng * params.odometrySigmaAng) : 0.0;
  if (wLin <= 0 && wAng <= 0) {
    return;
  }

  Eigen::Matrix<double, 6, 6> omega = Eigen::Matrix<double, 6, 6>::Zero();
  omega.topLeftCorner<3, 3>() = wLin * Mat3::Identity();
  omega.bottomRightCorner<3, 3>() = wAng * Mat3::Identity();

  constexpr double kEps = 1e-7;

  for (std::size_t k = 0; k < segments.size(); k++) {
    if (!segments[k].hasOdometry) {
      continue;
    }

    const SE3 & Ti = knots[k].state.T;
    const SE3 & Tj = knots[k + 1].state.T;
    const SE3 & measured = segments[k].odometryDelta;

    const Vec6 r = odometryResidual(Ti, Tj, measured);

    Eigen::Matrix<double, 6, 12> J;
    for (int i = 0; i < 12; i++) {
      Vec6 inc = Vec6::Zero();
      inc[i % 6] = kEps;
      SE3 pi = Ti;
      SE3 pj = Tj;
      incPose(i < 6 ? pi : pj, inc);

      inc[i % 6] = -kEps;
      SE3 mi = Ti;
      SE3 mj = Tj;
      incPose(i < 6 ? mi : mj, inc);

      J.col(i) =
        (odometryResidual(pi, pj, measured) - odometryResidual(mi, mj, measured)) / (2.0 * kEps);
    }

    const Mat12 H = J.transpose() * omega * J;
    const Vec12 g = -J.transpose() * omega * r;
    system_.addPosePairBlock(static_cast<int>(k), H, g);
  }
}

void WindowOptimizer::assemble(
  const std::vector<Knot> & knots, const std::vector<Segment> & segments,
  const MatchFunction & match, const MarginalizationPrior & prior, bool rematch, Result & result)
{
  const int dim = knotDim();
  const auto knotCount = static_cast<int>(knots.size());

  system_.setZero();
  lidarSystem_.setZero();
  result.chi2 = 0;
  result.errorSum = 0;
  result.inliers = 0;
  result.lidarPositionInfo = 0;
  result.imuPositionInfo = 0;
  result.priorPositionInfo = 0;
  result.lidarChi2 = 0;

  // Sums the position diagonal of a factor's Hessian over the knots it spans.
  const auto positionInfoOf = [](const auto & H, int knotsSpanned, int strideInBlock) {
    double sum = 0;
    for (int k = 0; k < knotsSpanned; k++) {
      for (int i = 0; i < 3; i++) {
        const int r = k * strideInBlock + kIdxPosition + i;
        sum += H(r, r);
      }
    }
    return sum;
  };

  // --- LiDAR ---
  for (std::size_t k = 0; k < segments.size(); k++) {
    if (segments[k].points.empty()) {
      continue;
    }

    const CtSegment current(knots[k].state.T, knots[k + 1].state.T);

    const bool atLinearizationPoint = knots[k].linearized || knots[k + 1].linearized;
    const CtSegment jacobianAt =
      atLinearizationPoint ? CtSegment(knots[k].jacobianState().T, knots[k + 1].jacobianState().T)
                           : current;

    if (rematch) {
      correspondences_[k].clear();
      match(k, current, segments[k].points, correspondences_[k]);
    }
    if (correspondences_[k].empty()) {
      continue;
    }

    const LidarBlock blk = assembleSegmentBlock(
      current, jacobianAt, segments[k].points, correspondences_[k], params.kernel,
      params.kernelScale);

    lidarSystem_.addPosePairBlock(static_cast<int>(k), blk.H, blk.g);

    result.lidarPositionInfo += positionInfoOf(blk.H, 2, 6);
    result.lidarChi2 += blk.chi2;
    result.errorSum += blk.errorSum;
    result.inliers += blk.inliers;
  }

  // Each correspondence supplies three residuals, and the window's own pose
  // freedoms are what the fit consumes.
  result.lidarDof = std::max(1.0, 3.0 * static_cast<double>(result.inliers) - 6.0 * knotCount);
  result.lidarScale = 1.0;

  if (params.lidarBalance != LidarBalance::None && result.lidarDof >= params.lidarBalanceMinDof) {
    double kappa = result.lidarChi2 / result.lidarDof;
    if (params.lidarBalance == LidarBalance::DownOnly) {
      kappa = std::max(1.0, kappa);
    }

    const double maxScale = std::max(1.0, params.lidarBalanceMaxScale);
    if (kappa > 0 && std::isfinite(kappa)) {
      result.lidarScale = std::clamp(1.0 / kappa, 1.0 / maxScale, maxScale);
    }
  }

  system_.H() += result.lidarScale * lidarSystem_.H();
  system_.g() += result.lidarScale * lidarSystem_.g();
  result.chi2 += result.lidarScale * result.lidarChi2;
  result.lidarPositionInfo *= result.lidarScale;

  // --- inertial, or the kinematic term that stands in for it ---
  if (params.useImu) {
    for (std::size_t k = 0; k < segments.size(); k++) {
      if (!segments[k].hasImu) {
        continue;
      }
      const ImuBlock imuBlk =
        assembleImuBlock(segments[k].imu, knots[k].state, knots[k + 1].state, params.gravity);
      system_.addStatePairBlock(static_cast<int>(k), imuBlk.H, imuBlk.g);
      result.imuPositionInfo += positionInfoOf(imuBlk.H, 2, kKnotDim);
      result.chi2 += imuBlk.chi2;

      const double dt = std::max(1e-6, knots[k + 1].t - knots[k].t);
      const ImuBlock rw = assembleBiasRandomWalkBlock(
        knots[k].state, knots[k + 1].state, dt, params.biasSigmaAcc, params.biasSigmaGyro);
      system_.addStatePairBlock(static_cast<int>(k), rw.H, rw.g);
      result.chi2 += rw.chi2;
    }

    for (int k = 0; k < knotCount; k++) {
      const KnotBlock bp =
        assembleBiasPriorBlock(knots[k].state, params.biasPriorSigmaAcc, params.biasPriorSigmaGyro);
      system_.addStateBlock(k, bp.H, bp.g);
      result.chi2 += bp.chi2;
    }
  } else {
    addTwistContinuity(knots);
  }

  addOdometry(knots, segments);

  // --- what the states that already left the window still have to say ---
  if (prior.valid && prior.H.rows() > 0) {
    const auto priorSize = static_cast<int>(prior.H.rows());
    const int priorKnots = priorSize / dim;

    Eigen::VectorXd deviation = Eigen::VectorXd::Zero(priorSize);
    for (int k = 0; k < priorKnots && k < knotCount; k++) {
      const Vec15 d = knotDeviation(knots[k].state, knots[k].priorAnchor);
      deviation.segment(k * dim, dim) = d.head(dim);
    }

    result.priorPositionInfo = positionInfoOf(prior.H, std::min(priorKnots, knotCount), dim);

    // The prior's gradient was taken where the states stood when it was built,
    // so it has to be re-centered on wherever they have moved to since.
    system_.addLeadingPrior(prior.H, prior.g - prior.H * deviation);
  }
}

WindowOptimizer::Result WindowOptimizer::optimize(
  std::vector<Knot> & knots, const std::vector<Segment> & segments, const MatchFunction & match,
  const MarginalizationPrior & prior)
{
  Result result;
  if (knots.size() < 2 || segments.size() + 1 != knots.size()) {
    return result;
  }

  const int dim = knotDim();
  const auto knotCount = static_cast<int>(knots.size());
  system_.resize(knotCount, dim);
  lidarSystem_.resize(knotCount, dim);

  correspondences_.assign(segments.size(), {});
  const int rematchEvery = std::max(1, params.rematchEvery);

  bool stepWasFinite = true;

  for (int iter = 0; iter < params.maxIterations; iter++) {
    const bool rematchedThisIteration = iter % rematchEvery == 0;

    assemble(knots, segments, match, prior, rematchedThisIteration, result);

    // --- solve and step ---
    Eigen::VectorXd step = system_.solve(params.lambda);
    if (!step.allFinite()) {
      stepWasFinite = false;
      break;
    }

    // Gauss-Newton is free to propose an arbitrarily long step when the system
    // is poorly conditioned. Scaling the whole step keeps its direction, which
    // a per-knot clamp would not, and lets the following iterations walk the
    // rest of the way if the direction was right after all.
    if (params.maxStepTranslation > 0) {
      double longest = 0;
      for (int k = 0; k < knotCount; k++) {
        longest = std::max(longest, step.segment<3>(k * dim + kIdxPosition).norm());
      }
      if (longest > params.maxStepTranslation) {
        step *= params.maxStepTranslation / longest;
        result.stepWasLimited = true;
      }
    }

    double maxTranslationStep = 0;
    for (int k = 0; k < knotCount; k++) {
      Vec15 inc = Vec15::Zero();
      inc.head(dim) = step.segment(k * dim, dim);
      incKnot(knots[k].state, inc);
      knots[k].state.T.normalize();

      maxTranslationStep = std::max(maxTranslationStep, inc.segment<3>(kIdxPosition).norm());
    }

    result.iterations = iter + 1;

    // Frozen pairings have an optimum of their own, and the trajectory reaches
    // it in a couple of iterations while still being far from where fresh
    // correspondences would put it. Convergence may therefore only be declared
    // on an iteration that actually re-matched, or the estimator stops early
    // and silently, which is exactly what it must not do.
    if (rematchedThisIteration && maxTranslationStep < params.convergenceThreshold) {
      result.converged = true;
      break;
    }
  }

  // The system a marginalization is taken from has to describe the states as
  // they finally stand, not as they stood one step earlier: the prior it
  // yields is declared to be a gradient at the final estimate, and any
  // mismatch there is a spurious force that the next window has no way to
  // tell from real information.
  if (stepWasFinite && result.iterations > 0) {
    assemble(knots, segments, match, prior, false, result);
  }

  return result;
}

}  // namespace mola::ct
