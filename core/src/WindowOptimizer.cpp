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

#include <Eigen/Eigenvalues>
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

  // The window's translational information, summed over knots. Translation
  // increments live in the world frame under this parameterization, so the
  // knots' blocks share an axis convention and adding them is meaningful.
  Mat3 lidarPositionBlock = Mat3::Zero();

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
    for (int j = 0; j < 2; j++) {
      const int r = j * 6 + kIdxPosition;
      lidarPositionBlock += blk.H.block<3, 3>(r, r);
    }
    result.lidarChi2 += blk.chi2;
    result.errorSum += blk.errorSum;
    result.inliers += blk.inliers;
  }

  // How evenly the geometry constrains the three translational axes. A view
  // that pins the position only within a plane, or only along a line, leaves
  // the correspondence count untouched while collapsing this, which is what
  // separates an uninformative view from a sparse one.
  {
    const Eigen::SelfAdjointEigenSolver<Mat3> eig(lidarPositionBlock);
    const Vec3 lambda = eig.eigenvalues();
    const double largest = lambda(2);
    const double smallest = lambda(0);
    result.lidarPositionWeakest = std::max(0.0, smallest);
    result.lidarPositionConditioning = largest > 0 ? std::clamp(smallest / largest, 0.0, 1.0) : 0.0;
    result.lidarWeakDirection = eig.eigenvectors().col(0);
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

    // A spike is bounded against what this sequence has recently looked like,
    // rather than against an absolute figure: the residual level is a
    // property of the scene, and only its sudden departures are suspect. The
    // bound is one-sided, see the parameter's documentation.
    if (params.lidarBalanceOutlierRatio > 0 && kappa > 0 && std::isfinite(kappa)) {
      const double baseline = kappaBaseline();
      if (baseline > 0) {
        kappa = std::min(kappa, params.lidarBalanceOutlierRatio * baseline);
      }
    }

    const double maxScale = std::max(1.0, params.lidarBalanceMaxScale);
    if (kappa > 0 && std::isfinite(kappa)) {
      result.lidarScaleInstant = std::clamp(1.0 / kappa, 1.0 / maxScale, maxScale);
    }

    // What is being estimated is a property of the sensor and of the scene's
    // statistics, and it moves slowly; one window's residuals do not.
    // Following them window by window hands the geometry's authority away
    // exactly when a transient makes the residuals large, which is when it is
    // needed most.
    result.lidarScale = params.lidarBalanceSmoothing > 0 && smoothedLidarScale_ > 0
                          ? smoothedLidarScale_
                          : result.lidarScaleInstant;
  }

  system_.H() += result.lidarScale * lidarSystem_.H();
  system_.g() += result.lidarScale * lidarSystem_.g();
  result.chi2 += result.lidarScale * result.lidarChi2;
  result.lidarPositionInfo *= result.lidarScale;

  // --- inertial, or the kinematic term that stands in for it ---
  if (params.useImu) {
    // A segment can lack an inertial factor even when the estimator is
    // inertial: the stream can end before the LiDAR's does, or the factor can
    // be refused for not spanning its segment. Such a segment used to receive
    // nothing at all, leaving the window free to move in whatever direction
    // the geometry did not pin. The term that stands in for the IMU when
    // there is none stands in here too.
    bool everySegmentHasImu = true;
    for (const auto & seg : segments) {
      everySegmentHasImu = everySegmentHasImu && seg.hasImu;
    }
    if (!everySegmentHasImu) {
      addTwistContinuity(knots);
    }

    for (std::size_t k = 0; k < segments.size(); k++) {
      if (!segments[k].hasImu) {
        continue;
      }

      // The factor's deltas describe `imu.dt`; the states it connects are
      // `knots[k+1].t - knots[k].t` apart. When those differ the residual
      // reads a whole interval's motion as if it happened in part of one, and
      // the preintegration covariance shrinks with the interval, so the
      // shorter the factor the more confident it is. The front end already
      // refuses such a factor; this is the same statement made where both
      // quantities are actually in scope.
      const double span = knots[k + 1].t - knots[k].t;
      if (span > 0 && segments[k].imu.dt < params.minImuSpanRatio * span) {
        result.imuFactorsRefused++;
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
      const KnotBlock vp = assembleVelocityPriorBlock(knots[k].state, params.velocityPriorSigma);
      system_.addStateBlock(k, vp.H, vp.g);
      result.chi2 += vp.chi2;

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
    // The step is scaled as a whole, by whichever bound it strains most, so
    // that its direction survives. Position and velocity are checked
    // separately because they are not in the same units and a step can be
    // unremarkable in one while running away in the other.
    double shrink = 1.0;
    if (params.maxStepTranslation > 0) {
      double longest = 0;
      for (int k = 0; k < knotCount; k++) {
        longest = std::max(longest, step.segment<3>(k * dim + kIdxPosition).norm());
      }
      if (longest > params.maxStepTranslation) {
        shrink = std::min(shrink, params.maxStepTranslation / longest);
      }
    }
    if (params.useImu && params.maxStepVelocity > 0) {
      double fastest = 0;
      for (int k = 0; k < knotCount; k++) {
        fastest = std::max(fastest, step.segment<3>(k * dim + kIdxVelocity).norm());
      }
      if (fastest > params.maxStepVelocity) {
        shrink = std::min(shrink, params.maxStepVelocity / fastest);
      }
    }
    if (shrink < 1.0) {
      step *= shrink;
      result.stepWasLimited = true;
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

  // The balance moves once per window, not once per iteration: it describes
  // the data rather than the state of the search.
  if (params.lidarBalance != LidarBalance::None && result.lidarScaleInstant > 0) {
    const double a = std::clamp(params.lidarBalanceSmoothing, 0.0, 1.0);
    smoothedLidarScale_ = smoothedLidarScale_ > 0 && a > 0
                            ? (1.0 - a) * smoothedLidarScale_ + a * result.lidarScaleInstant
                            : result.lidarScaleInstant;
  }

  // The baseline the clamp is measured against records what the window asked
  // for, not what it was allowed, so that a long stretch of genuinely harder
  // scenes still moves the baseline and the clamp follows it.
  if (params.lidarBalanceOutlierRatio > 0 && result.lidarDof > 0) {
    const double kappa = result.lidarChi2 / result.lidarDof;
    const auto capacity = static_cast<std::size_t>(std::max(1, params.lidarBalanceBaselineWindows));
    if (kappa > 0 && std::isfinite(kappa)) {
      if (recentKappa_.size() < capacity) {
        recentKappa_.push_back(kappa);
      } else {
        recentKappa_[recentKappaNext_ % capacity] = kappa;
      }
      recentKappaNext_++;
    }
  }

  return result;
}

/** Median of the recent per-window reduced chi-squares. A median rather than a
 * mean because the sequence it summarizes is exactly the one whose outliers
 * the clamp exists to reject. Zero until the buffer holds enough windows for
 * the median to mean anything.
 */
double WindowOptimizer::kappaBaseline() const
{
  constexpr std::size_t kMinWindowsForBaseline = 20;
  if (recentKappa_.size() < kMinWindowsForBaseline) {
    return 0;
  }
  std::vector<double> sorted = recentKappa_;
  const auto middle = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2);
  std::nth_element(sorted.begin(), middle, sorted.end());
  return *middle;
}

}  // namespace mola::ct
