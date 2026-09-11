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
 * @file   CtOdometryEngine.h
 * @brief  Turns timed points and IMU samples into a continuous trajectory
 */
#pragma once

#include <mola_ct_lio/WindowOptimizer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/system/CTimeLogger.h>

#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "CtMapMatcher.h"

namespace mola
{
/** The sliding-window estimator: it owns the knots, the segments, the map and
 * the marginalization prior, and turns a stream of timed points into a
 * trajectory.
 *
 * Poses are reported **once they leave the window**, i.e. after they have been
 * refined by every segment that could see them and then marginalized. A pose
 * therefore lags the newest data by about `knot_count * segment_interval`, and
 * comes out at the segment rate rather than the scan rate: there is no pose per
 * scan in a continuous-time method, and a scan may yield several or none.
 */
class CtOdometryEngine
{
public:
  CtOdometryEngine() = default;

  struct Params
  {
    /// Spacing of the control knots. [s]
    double segmentInterval = 0.04;

    /// Knots held in the window. One more than the number of segments.
    int knotCount = 4;

    /// Points outside this range band are dropped before anything else. [m]
    double minRange = 1.0;
    double maxRange = 100.0;

    /// Continuous-time noise densities of the IMU, used to propagate the
    /// preintegration covariance.
    double gyroNoiseDensity = 1.7e-4;
    /// Whether a knot's Jacobian linearization point is recaptured on every
    /// slide, or held at where the knot first entered a marginalization.
    /// Holding it is what keeps successive priors describing the same
    /// quantity; recapturing keeps the Jacobians closer to the current
    /// estimate. Which one wins is a property of the data, so it is a
    /// parameter rather than a decision.
    bool relinearizeEachSlide = false;

    double accelNoiseDensity = 2.0e-3;

    ct::WindowOptimizer::Params optimizer;
    CtMapMatcher::Params matcher;
  };

  Params params;

  /// A LiDAR return with an absolute timestamp, in the body frame.
  struct TimedPoint
  {
    ct::Vec3 p;
    double t = 0;
  };

  struct Diagnostics
  {
    int iterations = 0;
    bool converged = false;
    double chi2 = 0;
    std::size_t inliers = 0;
    std::size_t mapPoints = 0;
    std::size_t segmentPoints = 0;
    ct::Vec3 velocity = ct::Vec3::Zero();
    ct::Vec3 biasAcc = ct::Vec3::Zero();
    ct::Vec3 biasGyro = ct::Vec3::Zero();

    /// Whether the trust region had to shorten a step in this window.
    bool stepWasLimited = false;

    /// Total information the marginalization prior carries, and the size of
    /// the gradient it pulls with. A prior whose trace grows without bound is
    /// over-counting what the states that left the window actually knew, and
    /// makes the window stiff enough to stop following the data.
    double priorTrace = 0;
    double priorGradientNorm = 0;

    /// Position information contributed by each source. See
    /// WindowOptimizer::Result. [m^-2]
    double lidarPositionInfo = 0;
    double imuPositionInfo = 0;
    double priorPositionInfo = 0;
  };

  /// Called once per knot, when it leaves the window.
  std::function<void(double t, const ct::SE3 & pose, const Diagnostics &)> onPose;

  void initialize(const mrpt::containers::yaml & cfg);

  /** Sets where the trajectory starts and what the IMU biases are.
   *
   * The world frame must be gravity aligned for the IMU factors to mean
   * anything, so the initial attitude is not a free choice when the IMU is in
   * use: it comes from the initial calibration.
   */
  void setInitialState(
    const ct::SE3 & pose, const ct::Vec3 & biasAcc = ct::Vec3::Zero(),
    const ct::Vec3 & biasGyro = ct::Vec3::Zero());

  void addImuSample(double t, const ct::Vec3 & acc, const ct::Vec3 & gyro);

  /** Feeds one scan's worth of points. They need not be sorted. */
  void addPoints(const std::vector<TimedPoint> & points);

  /** Closes the trajectory, emitting the knots still inside the window.
   * Without it the trajectory is missing its last window. Idempotent.
   */
  void finish();

  [[nodiscard]] const CtMapMatcher & matcher() const { return matcher_; }

  /** Per-stage timings. Enabled by `profiler_enabled` in the YAML; the
   * breakdown is what says whether the cost is in the matching, the assembly
   * or the solve, which is not obvious from the outside.
   */
  mrpt::system::CTimeLogger profiler{false, "mola_ct_lio"};
  [[nodiscard]] std::size_t knotsEmitted() const { return knotsEmitted_; }

private:
  struct ImuSample
  {
    double t = 0;
    ct::Vec3 acc = ct::Vec3::Zero();
    ct::Vec3 gyro = ct::Vec3::Zero();
  };

  CtMapMatcher matcher_;
  ct::WindowOptimizer optimizer_;

  std::deque<ct::Knot> knots_;
  std::deque<ct::Segment> segments_;
  std::deque<ImuSample> imu_;
  std::vector<TimedPoint> pending_;

  ct::MarginalizationPrior prior_;
  ct::WindowOptimizer::Result lastResult_;

  bool started_ = false;
  bool finished_ = false;
  double epoch_ = 0;
  std::size_t knotsEmitted_ = 0;

  ct::SE3 initialPose_;
  ct::Vec3 initialBiasAcc_ = ct::Vec3::Zero();
  ct::Vec3 initialBiasGyro_ = ct::Vec3::Zero();

  void closeReadySegments(double latestPointTime);
  ct::Segment buildSegment(double tBegin, double tEnd);
  ct::KnotState predictNextKnot(const ct::Segment & segment) const;
  ct::PreintegratedImu preintegrate(double t0, double t1, const ct::KnotState & at) const;
  void optimizeWindow();
  void slideWindow();
  void emitOldest();
  void dropOldImuSamples(double before);
};

}  // namespace mola
