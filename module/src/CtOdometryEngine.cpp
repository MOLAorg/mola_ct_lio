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
 * @file   CtOdometryEngine.cpp
 * @brief  Turns timed points and IMU samples into a continuous trajectory
 */
#include "CtOdometryEngine.h"

#include <mola_imu_preintegration/ImuPreintegrator.h>

#include <algorithm>
#include <cmath>

namespace mola
{
namespace
{
ct::Mat3 toEigen(const mrpt::math::CMatrixDouble33 & m) { return m.asEigen(); }

ct::Vec3 toEigen(const mrpt::math::TVector3D & v) { return ct::Vec3(v.x, v.y, v.z); }

mrpt::math::TVector3D toMrpt(const ct::Vec3 & v) { return {v.x(), v.y(), v.z()}; }
}  // namespace

void CtOdometryEngine::initialize(const mrpt::containers::yaml & cfg)
{
  const auto readDouble = [&](const char * key, double & target) {
    if (cfg.has(key)) {
      target = cfg[key].as<double>();
    }
  };
  const auto readInt = [&](const char * key, int & target) {
    if (cfg.has(key)) {
      target = cfg[key].as<int>();
    }
  };
  const auto readBool = [&](const char * key, bool & target) {
    if (cfg.has(key)) {
      target = cfg[key].as<bool>();
    }
  };

  readDouble("segment_interval", params.segmentInterval);
  readInt("knot_count", params.knotCount);
  readDouble("min_range", params.minRange);
  readDouble("max_range", params.maxRange);
  readDouble("gyro_noise_density", params.gyroNoiseDensity);
  readDouble("accel_noise_density", params.accelNoiseDensity);
  readBool("relinearize_each_slide", params.relinearizeEachSlide);

  readInt("max_iterations", params.optimizer.maxIterations);
  readInt("rematch_every", params.optimizer.rematchEvery);
  readDouble("convergence_threshold", params.optimizer.convergenceThreshold);
  readDouble("lambda", params.optimizer.lambda);
  readDouble("max_step_translation", params.optimizer.maxStepTranslation);
  readDouble("kernel_scale", params.optimizer.kernelScale);
  readBool("use_imu", params.optimizer.useImu);
  readDouble("bias_sigma_acc", params.optimizer.biasSigmaAcc);
  readDouble("bias_sigma_gyro", params.optimizer.biasSigmaGyro);
  readDouble("twist_continuity_weight", params.optimizer.twistContinuityWeight);

  if (cfg.has("profiler_enabled")) {
    profiler.enable(cfg["profiler_enabled"].as<bool>());
  }

  if (cfg.has("kernel")) {
    const auto name = cfg["kernel"].as<std::string>();
    if (name == "None") {
      params.optimizer.kernel = ct::RobustKernel::None;
    } else if (name == "GemanMcClure") {
      params.optimizer.kernel = ct::RobustKernel::GemanMcClure;
    } else {
      params.optimizer.kernel = ct::RobustKernel::Cauchy;
    }
  }

  matcher_.params = params.matcher;
  matcher_.profiler = &profiler;
  matcher_.initialize(cfg);
  params.matcher = matcher_.params;

  optimizer_.params = params.optimizer;
}

void CtOdometryEngine::setInitialState(
  const ct::SE3 & pose, const ct::Vec3 & biasAcc, const ct::Vec3 & biasGyro)
{
  initialPose_ = pose;
  initialBiasAcc_ = biasAcc;
  initialBiasGyro_ = biasGyro;
}

void CtOdometryEngine::addImuSample(double t, const ct::Vec3 & acc, const ct::Vec3 & gyro)
{
  imu_.push_back(ImuSample{t, acc, gyro});
}

void CtOdometryEngine::dropOldImuSamples(double before)
{
  // One sample before the cut is kept, since integration holds each reading
  // constant over the interval that follows it.
  while (imu_.size() >= 2 && imu_[1].t < before) {
    imu_.pop_front();
  }
}

void CtOdometryEngine::addPoints(const std::vector<TimedPoint> & points)
{
  if (finished_ || points.empty()) {
    return;
  }

  const double minRangeSqr = params.minRange * params.minRange;
  const double maxRangeSqr = params.maxRange * params.maxRange;

  double latest = -std::numeric_limits<double>::infinity();
  for (const auto & p : points) {
    const double rangeSqr = p.p.squaredNorm();
    if (rangeSqr < minRangeSqr || rangeSqr > maxRangeSqr) {
      continue;
    }
    pending_.push_back(p);
    latest = std::max(latest, p.t);
  }

  if (pending_.empty()) {
    return;
  }

  if (!started_) {
    double earliest = std::numeric_limits<double>::infinity();
    for (const auto & p : pending_) {
      earliest = std::min(earliest, p.t);
    }
    epoch_ = earliest;
    started_ = true;

    ct::Knot first;
    first.t = epoch_;
    first.state.T = initialPose_;
    first.state.biasAcc = initialBiasAcc_;
    first.state.biasGyro = initialBiasGyro_;
    knots_.push_back(first);
  }

  closeReadySegments(latest);
}

ct::Segment CtOdometryEngine::buildSegment(double tBegin, double tEnd)
{
  ct::Segment seg;
  const double span = std::max(1e-9, tEnd - tBegin);

  std::vector<ct::SegmentPoint> raw;
  raw.reserve(pending_.size() / 2);
  for (const auto & p : pending_) {
    if (p.t < tBegin || p.t >= tEnd) {
      continue;
    }
    ct::SegmentPoint sp;
    sp.p = p.p;
    sp.alpha = (p.t - tBegin) / span;
    raw.push_back(sp);
  }

  seg.points = CtMapMatcher::downsample(raw, params.matcher.sourceVoxelSize);
  return seg;
}

ct::PreintegratedImu CtOdometryEngine::preintegrate(
  double t0, double t1, const ct::KnotState & at) const
{
  ct::PreintegratedImu out;

  mola::imu::ImuIntegrationParams p;
  p.gravity_vector = toMrpt(params.optimizer.gravity);
  p.cov_gyro = mola::imu::ImuIntegrationParams::isotropic_cov(params.gyroNoiseDensity);
  p.cov_acc = mola::imu::ImuIntegrationParams::isotropic_cov(params.accelNoiseDensity);

  mola::imu::ImuPreintegrator integrator(p);
  integrator.reset_integration(toMrpt(at.biasAcc), toMrpt(at.biasGyro));

  std::size_t used = 0;
  for (std::size_t i = 0; i + 1 < imu_.size(); i++) {
    const double sampleBegin = std::max(imu_[i].t, t0);
    const double sampleEnd = std::min(imu_[i + 1].t, t1);
    const double dt = sampleEnd - sampleBegin;
    if (dt <= 0) {
      continue;
    }
    integrator.integrate_measurement(toMrpt(imu_[i].acc), toMrpt(imu_[i].gyro), dt);
    used++;
  }

  if (used == 0) {
    return out;
  }

  const auto & s = integrator.current_state();
  out.dt = s.deltaTij;
  out.dR = toEigen(s.deltaRij);
  out.dV = toEigen(s.deltaVij);
  out.dP = toEigen(s.deltaPij);
  out.dR_dbg = toEigen(s.dR_dbg);
  out.dV_dba = toEigen(s.dV_dba);
  out.dV_dbg = toEigen(s.dV_dbg);
  out.dP_dba = toEigen(s.dP_dba);
  out.dP_dbg = toEigen(s.dP_dbg);
  out.cov = s.cov.asEigen();
  out.biasAcc = toEigen(s.bias_acc);
  out.biasGyro = toEigen(s.bias_gyro);
  return out;
}

ct::KnotState CtOdometryEngine::predictNextKnot(const ct::Segment & segment) const
{
  const ct::Knot & last = knots_.back();
  ct::KnotState next = last.state;

  if (params.optimizer.useImu && segment.hasImu) {
    // The IMU is the better predictor whenever it is there, and it is the only
    // one that can supply a velocity.
    const auto & pim = segment.imu;
    const double dt = pim.dt;
    next.T.R = last.state.T.R * pim.dR;
    next.v = last.state.v + params.optimizer.gravity * dt + last.state.T.R * pim.dV;
    next.T.t = last.state.T.t + last.state.v * dt + 0.5 * params.optimizer.gravity * dt * dt +
               last.state.T.R * pim.dP;
    next.T.normalize();
    return next;
  }

  // Otherwise repeat the last relative motion, which is exact for the constant
  // twist the trajectory model assumes anyway.
  if (knots_.size() >= 2) {
    const ct::SE3 & previous = knots_[knots_.size() - 2].state.T;
    const ct::SE3 relative = previous.inverse() * last.state.T;
    next.T = last.state.T * relative;
    next.T.normalize();
  }
  return next;
}

void CtOdometryEngine::closeReadySegments(double latestPointTime)
{
  while (true) {
    const double tBegin = knots_.back().t;
    const double tEnd = tBegin + params.segmentInterval;

    // A segment can only be closed once data beyond its end has arrived, or
    // its last points would be missing.
    if (latestPointTime < tEnd) {
      break;
    }

    ct::Segment seg;
    {
      mrpt::system::CTimeLoggerEntry tle(profiler, "buildSegment");
      seg = buildSegment(tBegin, tEnd);
    }

    if (params.optimizer.useImu) {
      mrpt::system::CTimeLoggerEntry tle(profiler, "preintegrate");
      seg.imu = preintegrate(tBegin, tEnd, knots_.back().state);
      seg.hasImu = seg.imu.dt > 0;
    }

    ct::Knot next;
    next.t = tEnd;
    next.state = predictNextKnot(seg);

    segments_.push_back(std::move(seg));
    knots_.push_back(next);

    pending_.erase(
      std::remove_if(
        pending_.begin(), pending_.end(), [tEnd](const TimedPoint & p) { return p.t < tEnd; }),
      pending_.end());
    dropOldImuSamples(tEnd);

    if (static_cast<int>(knots_.size()) > params.knotCount) {
      optimizeWindow();
      slideWindow();
    }
  }
}

void CtOdometryEngine::optimizeWindow()
{
  if (matcher_.empty()) {
    // Nothing to register against yet: seed the map from the oldest segment at
    // its predicted pose, which is the trajectory's own origin.
    return;
  }

  std::vector<ct::Knot> knots(knots_.begin(), knots_.end());
  std::vector<ct::Segment> segments(segments_.begin(), segments_.end());

  optimizer_.params = params.optimizer;

  mrpt::system::CTimeLoggerEntry tleOpt(profiler, "optimizeWindow");
  const auto result = optimizer_.optimize(
    knots, segments,
    [this](
      std::size_t, const ct::CtSegment & seg, const std::vector<ct::SegmentPoint> & points,
      std::vector<ct::PointCorrespondence> & out) {
      mrpt::system::CTimeLoggerEntry tle(profiler, "optimizeWindow.match");
      matcher_.match(seg, points, out);
    },
    prior_);
  tleOpt.stop();

  for (std::size_t i = 0; i < knots.size(); i++) {
    knots_[i] = knots[i];
  }

  lastResult_ = result;
}

void CtOdometryEngine::slideWindow()
{
  if (!matcher_.empty()) {
    mrpt::system::CTimeLoggerEntry tle(profiler, "marginalize");
    prior_ = ct::marginalizeLeadingKnots(optimizer_.lastSystem(), 1);
  }

  emitOldest();

  const ct::CtSegment oldest(knots_[0].state.T, knots_[1].state.T);
  {
    mrpt::system::CTimeLoggerEntry tle(profiler, "mapInsert");
    matcher_.insert(oldest, segments_[0].points);
  }

  knots_.pop_front();
  segments_.pop_front();

  // Every knot the prior speaks about is pinned from here on. The Jacobian
  // point is captured only the first time, so a knot keeps the same one for
  // its whole life in the window; the prior's own anchor moves with each new
  // prior, since that is where its gradient was just evaluated.
  for (auto & k : knots_) {
    if (!k.linearized || params.relinearizeEachSlide) {
      k.linearized = true;
      k.linearizationPoint = k.state;
    }
    k.priorAnchor = k.state;
  }
}

void CtOdometryEngine::emitOldest()
{
  if (!onPose) {
    return;
  }

  Diagnostics d;
  d.iterations = lastResult_.iterations;
  d.converged = lastResult_.converged;
  d.chi2 = lastResult_.chi2;
  d.inliers = lastResult_.inliers;
  d.mapPoints = matcher_.pointCount();
  d.segmentPoints = segments_[0].points.size();
  d.velocity = knots_[0].state.v;
  d.biasAcc = knots_[0].state.biasAcc;
  d.biasGyro = knots_[0].state.biasGyro;

  onPose(knots_[0].t, knots_[0].state.T, d);
  knotsEmitted_++;
}

void CtOdometryEngine::finish()
{
  if (finished_) {
    return;
  }
  finished_ = true;

  // Everything still inside the window has been refined as far as it can be,
  // so it is emitted rather than dropped.
  while (segments_.size() >= 1) {
    if (!matcher_.empty()) {
      optimizeWindow();
    }
    slideWindow();
  }
}

}  // namespace mola
