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

    /// A segment holding this fraction of what recent segments held is
    /// withheld from the map, on the reasoning that its pose is poorly
    /// determined and its points would mislead every scan that follows.
    ///
    /// Off by default, because measuring it says the reasoning is wrong: the
    /// points are worth more than the pose error they carry. Withholding them
    /// costs a healthy mission a factor of seven (0.022 m to 0.149 m) and
    /// makes both of the missions it was built for worse. Kept, disabled, so
    /// the measurement is not lost and the idea is not tried again blind.
    double starvationRatio = 0.0;

    /// Odometry samples older than the window are dropped; this is how much
    /// margin is kept so a segment can still interpolate across its own
    /// bounds. [s]
    double odometryKeepMargin = 1.0;

    /// Where in a segment the first scan is made to land, as a fraction of
    /// the segment. The LiDAR information of a point at alpha splits between
    /// the two knots as (1 - alpha) and alpha, so a scan sitting exactly on a
    /// segment boundary gives the end knot nothing. That costs little when the
    /// points of a scan spread across the segment anyway, and everything when
    /// a provider has already motion-compensated them and they all share one
    /// instant. Half a segment puts an undivided scan squarely between its two
    /// knots. [fraction of segment_interval]
    /// Zero keeps the grid anchored on the first measurement, which is what
    /// a scan whose points already span the segment wants. It is only worth
    /// moving for a provider that hands over one instant per scan.
    double segmentPhaseOffset = 0.0;

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

    /// See Segment::alphaSpread.
    double alphaSpread = 0;

    /// Whether this segment was held out of the map as starved.
    bool starved = false;
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

    /// See WindowOptimizer::Result.
    double lidarChi2 = 0;
    double lidarDof = 0;
    double lidarScale = 1.0;
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

  /** Feeds a pose from an external odometry source, in that source's own
   * fixed frame. Only relative motion between two instants is ever used, so
   * the frame's origin and drift do not matter.
   */
  void addOdometrySample(double t, const ct::SE3 & pose);

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

  /// A slow average of how many points a segment has been holding, which is
  /// what a starved one is judged against.
  double pointCountAverage_ = 0;
  std::size_t starvedSegments_ = 0;

  struct OdometrySample
  {
    double t = 0;
    ct::SE3 pose;
  };
  std::deque<OdometrySample> odometry_;

  /** The motion an external odometry reports between two instants, expressed
   * in the frame it had at `t0`. False when the samples do not bracket both.
   */
  [[nodiscard]] bool odometryDelta(double t0, double t1, ct::SE3 & out) const;

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
