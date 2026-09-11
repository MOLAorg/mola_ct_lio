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
 * @file   CtLidarInertialOdometry.cpp
 * @brief  MOLA front end: continuous-time LiDAR-inertial odometry
 */
#include <mola_ct_lio/CtLidarInertialOdometry.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/Lie/SO.h>

#include <cstdlib>
#include <fstream>

#include "CtOdometryEngine.h"
#include "ScanAdapters.h"

IMPLEMENTS_MRPT_OBJECT(CtLidarInertialOdometry, mola::FrontEndBase, mola)

namespace mola
{
namespace
{
/// How many observations may go by with no scan before it is worth saying so.
constexpr std::size_t kObservationsBeforeComplaining = 2000;

/// How many segments to watch before judging whether their points span enough
/// of each segment for the interpolation to be identifiable.
constexpr std::size_t kSegmentsBeforeJudgingSpread = 200;

/// Below this fraction of a segment, the points are effectively all at one
/// instant and the segment's end knot gets no LiDAR information.
constexpr double kUsableAlphaSpread = 0.1;

/** Opens a diagnostic stream named by an environment variable, or returns
 * null. These exist so that the inertial path can be checked against a
 * ground-truth trajectory instead of assumed correct: a wrong extrinsic or a
 * wrong frame convention does not fail, it just degrades the result.
 */
std::unique_ptr<std::ofstream> openDumpStream(const char * envVar, const char * header)
{
  const char * path = ::getenv(envVar);
  if (!path) {
    return {};
  }
  auto f = std::make_unique<std::ofstream>(path);
  if (!f->is_open()) {
    return {};
  }
  *f << header << "\n";
  return f;
}

mrpt::poses::CPose3D toMrptPose(const ct::SE3 & p)
{
  mrpt::math::CMatrixDouble33 R;
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      R(r, c) = p.R(r, c);
    }
  }

  mrpt::poses::CPose3D out;
  out.setRotationMatrix(R);
  out.x(p.t[0]);
  out.y(p.t[1]);
  out.z(p.t[2]);
  return out;
}
}  // namespace

CtLidarInertialOdometry::CtLidarInertialOdometry() : engine_(std::make_unique<CtOdometryEngine>())
{
  this->mrpt::system::COutputLogger::setLoggerName("CtLidarInertialOdometry");
}

CtLidarInertialOdometry::~CtLidarInertialOdometry() { finish(); }

std::size_t CtLidarInertialOdometry::posesEmitted() const { return engine_->knotsEmitted(); }

void CtLidarInertialOdometry::initialize_frontend(const Yaml & c)
{
  MRPT_START

  auto cfg = c;
  if (cfg.has("params")) {
    cfg = cfg["params"];
  }

  YAML_LOAD_OPT(lidar_sensor_label, std::string);
  YAML_LOAD_OPT(imu_sensor_label, std::string);
  YAML_LOAD_OPT(odometry_sensor_label, std::string);
  YAML_LOAD_OPT(imu_time_offset, double);
  YAML_LOAD_OPT(baselink2lidar_pose_str, std::string);
  YAML_LOAD_OPT(fallback_scan_period, double);
  YAML_LOAD_OPT(map_publish_period, double);
  YAML_LOAD_OPT(max_initial_gyro_bias, double);
  YAML_LOAD_OPT(max_initial_accel_bias, double);
  YAML_LOAD_OPT(use_observation_sensor_pose, bool);
  YAML_LOAD_OPT(clouds_already_deskewed, bool);

  lidar_sensor_label_regex_ = std::regex(lidar_sensor_label);
  imu_sensor_label_regex_ = std::regex(imu_sensor_label);
  odometry_sensor_label_regex_ = std::regex(odometry_sensor_label);

  lidar_pose_in_baselink_ = mrpt::poses::CPose3D::FromString("[" + baselink2lidar_pose_str + "]");

  engine_->initialize(mrpt::containers::yaml(cfg));

  if (!engine_->params.optimizer.useImu) {
    // No inertial data means no gravity-aligned world frame to establish, so
    // the trajectory simply starts at the origin.
    engine_->setInitialState(ct::SE3());
    imu_initialized_ = true;
  }

  MRPT_LOG_INFO_STREAM(
    "Initialized: segment_interval=" << engine_->params.segmentInterval
                                     << " knot_count=" << engine_->params.knotCount
                                     << " use_imu=" << engine_->params.optimizer.useImu
                                     << " lidar_pose_in_baselink=" << lidar_pose_in_baselink_);

  imu_dump_ = openDumpStream("MOLA_CTLIO_DUMP_IMU", "# t wx wy wz ax ay az  (body frame)");
  state_dump_ = openDumpStream(
    "MOLA_CTLIO_DUMP_STATE",
    "# t x y z vx vy vz bax bay baz bgx bgy bgz inliers chi2 segpts iters conv "
    "limited priorTrace priorGrad lidarPosInfo imuPosInfo priorPosInfo lidarChi2 "
    "lidarDof lidarScale starved");

  engine_->onPose = [this](
                      double t, const ct::SE3 & pose, const CtOdometryEngine::Diagnostics & d) {
    if (d.alphaSpread < kUsableAlphaSpread) {
      degenerate_segments_++;
    }
    if (
      ++segments_seen_ == kSegmentsBeforeJudgingSpread && !warned_degenerate_segments_ &&
      degenerate_segments_ * 2 > segments_seen_) {
      warned_degenerate_segments_ = true;
      MRPT_LOG_WARN_STREAM(
        "Most segments so far hold points spanning under "
        << kUsableAlphaSpread
        << " of their own interval, so the continuous-time interpolation has "
           "nothing to fit and each segment's end knot gets no LiDAR information. "
           "This happens when the clouds arrive already motion-compensated, since "
           "every point of a scan then shares one instant: a segment holding a "
           "single scan collapses to one alpha. Raise 'segment_interval' to an "
           "integer multiple of the scan period so each segment spans at least "
           "two scans. It drifts rather than fails, so nothing else will say so.");
    }

    if (state_dump_) {
      *state_dump_ << mrpt::format(
        "%.6f %.4f %.4f %.4f %.4f %.4f %.4f %.6f %.6f %.6f %.6f %.6f %.6f %zu %.4e %zu %d %d "
        "%d %.6e %.6e %.6e %.6e %.6e %.6e %.6e %.6e %d\n",
        t, pose.t.x(), pose.t.y(), pose.t.z(), d.velocity.x(), d.velocity.y(), d.velocity.z(),
        d.biasAcc.x(), d.biasAcc.y(), d.biasAcc.z(), d.biasGyro.x(), d.biasGyro.y(), d.biasGyro.z(),
        d.inliers, d.chi2, d.segmentPoints, d.iterations, d.converged ? 1 : 0,
        d.stepWasLimited ? 1 : 0, d.priorTrace, d.priorGradientNorm, d.lidarPositionInfo,
        d.imuPositionInfo, d.priorPositionInfo, d.lidarChi2, d.lidarDof, d.lidarScale,
        d.starved ? 1 : 0);
    }
    publishPose(t, toMrptPose(pose));
  };

  MRPT_END
}

void CtLidarInertialOdometry::onNewObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  if (!o) {
    return;
  }

  observations_seen_++;

  // A dataset whose LiDAR observations never arrive looks exactly like one
  // that is simply quiet, and the run then ends with an empty trajectory and
  // no reason given. The usual cause is a bag with no /tf, where the reader
  // drops every cloud it cannot resolve a sensor pose for.
  if (
    scans_processed_ == 0 && observations_seen_ > kObservationsBeforeComplaining &&
    !warned_no_scans_) {
    warned_no_scans_ = true;
    MRPT_LOG_WARN_STREAM(
      "Seen " << observations_seen_
              << " observations and not one LiDAR scan. Check that the sensor label matches '"
              << lidar_sensor_label
              << "', and that the dataset source can resolve a pose for the cloud: a bag with "
                 "no /tf needs a fixed sensor pose to be configured, or every scan is dropped "
                 "before it gets here.");
  }

  if (std::regex_match(o->sensorLabel, imu_sensor_label_regex_)) {
    onImu(o);
    return;
  }
  if (std::regex_match(o->sensorLabel, odometry_sensor_label_regex_)) {
    onOdometry(o);
    return;
  }
  if (std::regex_match(o->sensorLabel, lidar_sensor_label_regex_)) {
    onLidar(o);
  }
}

/** Takes a pose from an external odometry source.
 *
 * Both shapes the datasets use are accepted: the full SE(3)
 * CObservationRobotPose, and the planar CObservationOdometry, which carries
 * only x, y and yaw. The planar one is read as a pose at zero height and
 * level, so a platform that climbs stairs must not be fed through it.
 */
void CtLidarInertialOdometry::onOdometry(const mrpt::obs::CObservation::ConstPtr & o)
{
  mrpt::poses::CPose3D pose;

  if (auto rp = std::dynamic_pointer_cast<const mrpt::obs::CObservationRobotPose>(o); rp) {
    pose = rp->pose.mean;
  } else if (auto od = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o); od) {
    pose = mrpt::poses::CPose3D(od->odometry);
  } else {
    return;
  }

  ct::SE3 p;
  p.R = pose.getRotationMatrix().asEigen();
  p.t = ct::Vec3(pose.x(), pose.y(), pose.z());

  engine_->addOdometrySample(mrpt::Clock::toDouble(o->timestamp), p);
  odometry_samples_++;
}

void CtLidarInertialOdometry::onImu(const mrpt::obs::CObservation::ConstPtr & o)
{
  if (!engine_->params.optimizer.useImu) {
    return;
  }

  auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o);
  if (!imu) {
    return;
  }

  // Readings must reach the estimator in the body frame, lever arm included.
  const auto inBody = imu_transformer_.process(*imu);

  const double t = mrpt::Clock::toDouble(inBody.timestamp);

  const bool hasAcc = inBody.has(mrpt::obs::IMU_X_ACC) && inBody.has(mrpt::obs::IMU_Y_ACC) &&
                      inBody.has(mrpt::obs::IMU_Z_ACC);
  const bool hasGyro =
    inBody.has(mrpt::obs::IMU_WX) && inBody.has(mrpt::obs::IMU_WY) && inBody.has(mrpt::obs::IMU_WZ);
  if (!hasAcc || !hasGyro) {
    return;
  }

  const ct::Vec3 acc(
    inBody.get(mrpt::obs::IMU_X_ACC), inBody.get(mrpt::obs::IMU_Y_ACC),
    inBody.get(mrpt::obs::IMU_Z_ACC));
  const ct::Vec3 gyro(
    inBody.get(mrpt::obs::IMU_WX), inBody.get(mrpt::obs::IMU_WY), inBody.get(mrpt::obs::IMU_WZ));

  if (imu_dump_) {
    *imu_dump_ << mrpt::format(
      "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", t, gyro.x(), gyro.y(), gyro.z(), acc.x(), acc.y(),
      acc.z());
  }

  if (!imu_initialized_) {
    imu_calibrator_.add(std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(
      mrpt::obs::CObservationIMU::Create(inBody)));

    const auto calibration = imu_calibrator_.getCalibration();
    if (!calibration) {
      return;
    }

    // The world frame has to be gravity aligned for the inertial factors to
    // mean anything, so the initial attitude is not a free choice: it is the
    // levelling the calibration measured. Yaw stays at zero, being
    // unobservable from gravity alone.
    const auto initialPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
      0.0, 0.0, 0.0, 0.0, calibration->pitch, calibration->roll);

    ct::SE3 pose;
    const auto R = initialPose.getRotationMatrix();
    for (int r = 0; r < 3; r++) {
      for (int cc = 0; cc < 3; cc++) {
        pose.R(r, cc) = R(r, cc);
      }
    }

    // A bias is only a bias if the platform was still while it was measured,
    // and the readiness gate cannot tell that on its own: it accepts on the
    // steadiness of the accelerometer direction, which a platform turning
    // about the gravity axis satisfies perfectly while the gyroscope reads its
    // rotation. Averaged over such a window the "bias" is the motion, and
    // seeding it is worse than seeding nothing, because the random walk
    // between knots is deliberately tight and a wrong seed takes far longer to
    // walk off than a sequence lasts.
    //
    // What does separate the two is magnitude: a real MEMS bias is small, so a
    // measured one beyond a plausible bound is rejected outright.
    const auto readiness = imu_calibrator_.readiness();

    const ct::Vec3 measuredBiasAcc(
      calibration->bias_acc_b.x, calibration->bias_acc_b.y, calibration->bias_acc_b.z);
    const ct::Vec3 measuredBiasGyro(
      calibration->bias_gyro.x, calibration->bias_gyro.y, calibration->bias_gyro.z);

    const bool gyroPlausible =
      !readiness.timed_out && measuredBiasGyro.norm() <= max_initial_gyro_bias;
    const bool accelPlausible =
      !readiness.timed_out && measuredBiasAcc.norm() <= max_initial_accel_bias;

    const ct::Vec3 biasGyro = gyroPlausible ? measuredBiasGyro : ct::Vec3::Zero();
    const ct::Vec3 biasAcc = accelPlausible ? measuredBiasAcc : ct::Vec3::Zero();

    engine_->setInitialState(pose, biasAcc, biasGyro);

    imu_initialized_ = true;
    MRPT_LOG_INFO_STREAM(
      "IMU initialized: pitch=" << mrpt::RAD2DEG(calibration->pitch)
                                << " deg roll=" << mrpt::RAD2DEG(calibration->roll)
                                << " deg measured bias_gyro=" << calibration->bias_gyro.asString()
                                << " dispersion="
                                << (readiness.dispersion ? *readiness.dispersion : -1.0) << " rad");
    if (!gyroPlausible || !accelPlausible) {
      MRPT_LOG_WARN_STREAM(
        "Rejecting the measured IMU bias as implausible (gyro "
        << measuredBiasGyro.norm() << " rad/s against a bound of " << max_initial_gyro_bias
        << ", accel " << measuredBiasAcc.norm() << " m/s2 against " << max_initial_accel_bias
        << "). The platform was most likely moving while the calibration ran, so this is its "
           "motion rather than a bias. The attitude is kept and the biases are left to the "
           "estimator.");
    }
  }

  engine_->addImuSample(t + imu_time_offset, acc, gyro);
}

void CtLidarInertialOdometry::onLidar(const mrpt::obs::CObservation::ConstPtr & o)
{
  auto pc = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(o);
  if (!pc) {
    MRPT_LOG_DEBUG_STREAM(
      "Ignoring an observation matching the LiDAR label but not a point cloud, class="
      << o->GetRuntimeClass()->className);
    return;
  }

  if (!imu_initialized_) {
    // Registering against a world frame that is not levelled yet would put the
    // first of the map into the wrong place permanently, so scans are dropped
    // until the IMU has said which way is down.
    if (!warned_no_imu_) {
      MRPT_LOG_INFO("Waiting for the initial IMU calibration before processing scans");
      warned_no_imu_ = true;
    }
    return;
  }

  // The dataset source fills in the sensor pose, from /tf or from a fixed
  // configuration, and that is by definition the LiDAR in the body frame. The
  // YAML parameter is an *extra* transform composed on top of it, for the
  // datasets that cannot supply one: with a bag carrying /tf the parameter
  // stays at identity, and with a bag that has none the reader supplies
  // identity and the parameter carries the whole extrinsic. Getting this wrong
  // leaves the points in one frame and the inertial factors in another, which
  // does not fail, it just degrades the trajectory.
  mrpt::poses::CPose3D sensorInBody = lidar_pose_in_baselink_;
  if (use_observation_sensor_pose) {
    sensorInBody = lidar_pose_in_baselink_ + pc->sensorPose;
  }

  if (scans_processed_ == 0) {
    MRPT_LOG_INFO_STREAM(
      "LiDAR pose in the body frame: " << sensorInBody << " (observation says " << pc->sensorPose
                                       << ", parameter adds " << lidar_pose_in_baselink_ << ")");
  }

  ScanTimeSource timeSource = ScanTimeSource::AzimuthFallback;
  auto points =
    toTimedPoints(*pc, sensorInBody, fallback_scan_period, clouds_already_deskewed, timeSource);
  if (points.empty()) {
    MRPT_LOG_DEBUG("Ignoring a scan that converted to no points");
    return;
  }

  if (scans_processed_ == 0) {
    MRPT_LOG_INFO_STREAM("Per-point timing source: " << toString(timeSource));
  }

  engine_->addPoints(points);
  scans_processed_++;

  publishMap(pc->timestamp);
}

void CtLidarInertialOdometry::publishPose(double t, const mrpt::poses::CPose3D & pose)
{
  const auto timestamp = mrpt::Clock::fromDouble(t);

  {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    trajectory_.insert(timestamp, pose);
  }

  if (anyUpdateLocalizationSubscriber()) {
    LocalizationUpdate lu;
    lu.timestamp = timestamp;
    lu.reference_frame = "map";
    lu.child_frame = "base_link";
    lu.method = "ct-lio";
    lu.pose = pose.asTPose();
    advertiseUpdatedLocalization(lu);
  }

  updateVisualization(pose);
}

void CtLidarInertialOdometry::updateVisualization(const mrpt::poses::CPose3D & pose)
{
  if (!visualizer_) {
    return;
  }

  if (visualization_params_.current_pose_corner_size > 0) {
    auto glVehicle = mrpt::opengl::CSetOfObjects::Create();
    glVehicle->insert(
      mrpt::opengl::stock_objects::CornerXYZ(visualization_params_.current_pose_corner_size));
    glVehicle->setPose(pose);
    visualizer_->update_3d_object("ctlio/vehicle", glVehicle);
  }

  updateVisualizationPath(pose);
  updateVisualizationMap();
}

void CtLidarInertialOdometry::updateVisualizationPath(const mrpt::poses::CPose3D & pose)
{
  if (!visualization_params_.show_trajectory) {
    visualizer_->update_3d_object("ctlio/path", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  if (!gl_path_) {
    gl_path_ = mrpt::opengl::CSetOfLines::Create();
  }

  const auto t = pose.translation();
  if (gl_path_->empty()) {
    gl_path_->appendLine(t, t);
  } else {
    gl_path_->appendLineStrip(t);
  }

  auto grp = mrpt::opengl::CSetOfObjects::Create();
  grp->insert(mrpt::opengl::CSetOfLines::Create(*gl_path_));
  visualizer_->update_3d_object("ctlio/path", grp);
}

void CtLidarInertialOdometry::updateVisualizationMap()
{
  if (!visualization_params_.show_map) {
    visualizer_->update_3d_object("ctlio/map", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  if (++map_viz_counter_ < visualization_params_.map_update_decimation) {
    return;
  }
  map_viz_counter_ = 0;

  const auto & m = engine_->matcher().map();
  if (m.isEmpty()) {
    return;
  }

  auto points = m.liveCompactedCopy();
  if (!points || points->empty()) {
    return;
  }

  auto glCloud = mrpt::opengl::CPointCloudColoured::Create();
  glCloud->loadFromPointsMap(points.get());
  glCloud->setPointSize(visualization_params_.map_point_size);

  // Height is what makes a street or a staircase legible at a glance, and it
  // needs no extra channel from the estimator to compute.
  const auto bbox = points->boundingBox();
  glCloud->recolorizeByCoordinate(bbox.min.z, bbox.max.z, 2 /*Z*/, mrpt::img::TColormap::cmJET);

  auto grp = mrpt::opengl::CSetOfObjects::Create();
  grp->insert(glCloud);
  visualizer_->update_3d_object("ctlio/map", grp);
}

void CtLidarInertialOdometry::publishMap(const mrpt::Clock::time_point & timestamp)
{
  if (!anyUpdateMapSubscriber()) {
    return;
  }

  const double now = mrpt::Clock::toDouble(timestamp);
  if (now - last_map_publish_ < map_publish_period) {
    return;
  }
  last_map_publish_ = now;

  const auto & m = engine_->matcher().map();
  if (m.isEmpty()) {
    return;
  }

  MapUpdate mu;
  mu.timestamp = timestamp;
  mu.reference_frame = "map";
  mu.method = "ct-lio";
  mu.map_name = "local_map";
  mu.map = m.liveCompactedCopy();
  mu.keep_last_one_only = true;
  advertiseUpdatedMap(mu);
}

void CtLidarInertialOdometry::spinOnce() {}

void CtLidarInertialOdometry::finish()
{
  if (finished_) {
    return;
  }
  finished_ = true;
  engine_->finish();

  if (engine_->profiler.isEnabled()) {
    MRPT_LOG_INFO_STREAM("Stage timings:\n" << engine_->profiler.getStatsAsText());
  }
}

void CtLidarInertialOdometry::onQuit() { finish(); }

mrpt::poses::CPose3DInterpolator CtLidarInertialOdometry::estimatedTrajectory() const
{
  auto lck = mrpt::lockHelper(trajectory_mtx_);
  return trajectory_;
}

}  // namespace mola
