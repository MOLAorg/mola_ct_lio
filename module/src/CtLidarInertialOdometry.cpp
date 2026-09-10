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
#include <mrpt/core/lock_helper.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/Lie/SO.h>

#include "CtOdometryEngine.h"
#include "ScanAdapters.h"

IMPLEMENTS_MRPT_OBJECT(CtLidarInertialOdometry, mola::FrontEndBase, mola)

namespace mola
{
namespace
{
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
  YAML_LOAD_OPT(baselink2lidar_pose_str, std::string);
  YAML_LOAD_OPT(fallback_scan_period, double);
  YAML_LOAD_OPT(map_publish_period, double);

  lidar_sensor_label_regex_ = std::regex(lidar_sensor_label);
  imu_sensor_label_regex_ = std::regex(imu_sensor_label);

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

  engine_->onPose = [this](double t, const ct::SE3 & pose, const CtOdometryEngine::Diagnostics &) {
    publishPose(t, toMrptPose(pose));
  };

  MRPT_END
}

void CtLidarInertialOdometry::onNewObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  if (!o) {
    return;
  }

  if (std::regex_match(o->sensorLabel, imu_sensor_label_regex_)) {
    onImu(o);
    return;
  }
  if (std::regex_match(o->sensorLabel, lidar_sensor_label_regex_)) {
    onLidar(o);
  }
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

    engine_->setInitialState(
      pose,
      ct::Vec3(calibration->bias_acc_b.x, calibration->bias_acc_b.y, calibration->bias_acc_b.z),
      ct::Vec3(calibration->bias_gyro.x, calibration->bias_gyro.y, calibration->bias_gyro.z));

    imu_initialized_ = true;
    MRPT_LOG_INFO_STREAM(
      "IMU initialized: pitch=" << mrpt::RAD2DEG(calibration->pitch)
                                << " deg roll=" << mrpt::RAD2DEG(calibration->roll)
                                << " deg bias_gyro=" << calibration->bias_gyro.asString());
  }

  engine_->addImuSample(t, acc, gyro);
}

void CtLidarInertialOdometry::onLidar(const mrpt::obs::CObservation::ConstPtr & o)
{
  auto pc = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(o);
  if (!pc) {
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

  ScanTimeSource timeSource = ScanTimeSource::AzimuthFallback;
  auto points = toTimedPoints(*pc, lidar_pose_in_baselink_, fallback_scan_period, timeSource);
  if (points.empty()) {
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
}

void CtLidarInertialOdometry::onQuit() { finish(); }

mrpt::poses::CPose3DInterpolator CtLidarInertialOdometry::estimatedTrajectory() const
{
  auto lck = mrpt::lockHelper(trajectory_mtx_);
  return trajectory_;
}

}  // namespace mola
