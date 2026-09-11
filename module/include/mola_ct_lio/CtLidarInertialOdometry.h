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
 * @file   CtLidarInertialOdometry.h
 * @brief  MOLA front end: continuous-time LiDAR-inertial odometry
 */
#pragma once

#include <mola_imu_preintegration/ImuInitialCalibrator.h>
#include <mola_imu_preintegration/ImuTransformer.h>
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <string>

namespace mola
{
class CtOdometryEngine;

/** Continuous-time LiDAR-inertial odometry.
 *
 * The trajectory is a sliding window of control knots rather than one pose per
 * scan: each sweep is cut into segments, every point is registered at its own
 * timestamp, and the knots are optimized jointly with preintegrated IMU
 * factors between them.
 *
 * Two consequences are worth knowing before reading a trajectory it produced:
 *
 * - **Poses come out at the segment rate**, not the scan rate, so a given scan
 *   may yield several or none.
 * - **A pose is reported only once it leaves the window**, i.e. after it has
 *   been refined by every segment that could see it and then marginalized. The
 *   live trajectory therefore lags the newest scan by about one window.
 *
 * Everything is estimated in the **body frame**: the LiDAR extrinsic is
 * applied to the points as they arrive rather than composed onto the result
 * afterwards, because the IMU factors live in that frame and the two have to
 * agree. The reported trajectory is `map -> base_link` directly.
 *
 * \ingroup mola_ct_lio_grp
 */
class CtLidarInertialOdometry : public FrontEndBase,
                                public LocalizationSourceBase,
                                public MapSourceBase
{
  DEFINE_MRPT_OBJECT(CtLidarInertialOdometry, mola)

public:
  CtLidarInertialOdometry();
  ~CtLidarInertialOdometry() override;

  // ExecutableBase
  void spinOnce() override;
  void onQuit() override;

  // RawDataConsumer
  void onNewObservation(const mrpt::obs::CObservation::ConstPtr & o) override;

  /** Flushes the sliding window, emitting the knots still inside it. Without
   * this the trajectory is missing its last window. Idempotent.
   */
  void finish();

  /** Trajectory accumulated so far (`map` -> `base_link`). */
  [[nodiscard]] mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

  [[nodiscard]] std::size_t scansProcessed() const { return scans_processed_; }
  [[nodiscard]] std::size_t posesEmitted() const;

protected:
  void initialize_frontend(const Yaml & cfg) override;

private:
  void onLidar(const mrpt::obs::CObservation::ConstPtr & o);
  void onImu(const mrpt::obs::CObservation::ConstPtr & o);
  void publishPose(double t, const mrpt::poses::CPose3D & pose);
  void publishMap(const mrpt::Clock::time_point & timestamp);

  std::unique_ptr<CtOdometryEngine> engine_;

  std::regex lidar_sensor_label_regex_{"lidar"};
  std::regex imu_sensor_label_regex_{"imu"};

  std::string lidar_sensor_label = "lidar";
  std::string imu_sensor_label = "imu";

  /// Extra transform composed on top of the observation's own sensor pose,
  /// as "x y z yaw_deg pitch_deg roll_deg". It carries the whole extrinsic
  /// only for datasets whose reader cannot supply one.
  std::string baselink2lidar_pose_str = "0 0 0 0 0 0";

  /// Whether to take the LiDAR extrinsic from the observation, which is what
  /// a dataset source fills in from /tf or from its fixed-pose configuration.
  bool use_observation_sensor_pose = true;

  /// Used only when a sweep carries no usable per-point time field. [s]
  double fallback_scan_period = 0.1;

  /// Set for datasets whose provider already motion-compensated the clouds.
  /// See toTimedPoints(): such a cloud usually keeps its per-point time field,
  /// so the times look usable while the geometry is already corrected, and
  /// deskewing it again would double the correction.
  bool clouds_already_deskewed = false;

  /// Publish the map layer to subscribers at most this often. [s]
  double map_publish_period = 0.5;

  /// Largest initial bias that will be believed, beyond which the measurement
  /// is taken to be platform motion rather than a bias and is discarded.
  /// [rad/s] and [m/s^2].
  double max_initial_gyro_bias = 0.02;
  double max_initial_accel_bias = 0.5;

  mrpt::poses::CPose3D lidar_pose_in_baselink_;

  mola::imu::ImuInitialCalibrator imu_calibrator_;
  mola::imu::ImuTransformer imu_transformer_;
  bool imu_initialized_ = false;
  bool warned_no_imu_ = false;

  std::size_t scans_processed_ = 0;
  std::size_t observations_seen_ = 0;
  bool warned_no_scans_ = false;

  /** What the 3D view shows, when one is attached.
   *
   * Deliberately small: the path the estimator has walked and the map it is
   * registering against are what tell an operator whether a run is healthy,
   * and anything beyond that belongs in the diagnostics dump instead.
   */
  struct VisualizationParams
  {
    std::atomic_bool show_trajectory{true};
    std::atomic_bool show_map{true};

    float current_pose_corner_size = 1.0f;
    float map_point_size = 2.0f;

    /// Processed scans between map refreshes. Redrawing a large cloud costs
    /// more than the odometry step it would be reporting on.
    int map_update_decimation = 10;
  } visualization_params_;

  mrpt::opengl::CSetOfLines::Ptr gl_path_;
  int map_viz_counter_ = 0;

  void updateVisualization(const mrpt::poses::CPose3D & pose);
  void updateVisualizationPath(const mrpt::poses::CPose3D & pose);
  void updateVisualizationMap();

  /// Segments whose points span too little of their own interval for the
  /// continuous-time interpolation to be identifiable, counted over the first
  /// few hundred so the run can say so once rather than silently drift.
  std::size_t segments_seen_ = 0;
  std::size_t degenerate_segments_ = 0;
  bool warned_degenerate_segments_ = false;
  double last_map_publish_ = 0;

  mutable std::mutex trajectory_mtx_;
  mrpt::poses::CPose3DInterpolator trajectory_;

  bool finished_ = false;

  /// Optional diagnostic streams, see openDumpStream() in the .cpp.
  std::unique_ptr<std::ofstream> imu_dump_;
  std::unique_ptr<std::ofstream> state_dump_;
};

}  // namespace mola
