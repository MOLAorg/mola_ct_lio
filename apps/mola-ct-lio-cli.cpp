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
 * @file   mola-ct-lio-cli.cpp
 * @brief  main() for the offline, loss-free continuous-time LiDAR-inertial CLI.
 *
 * Same dataset-source helpers and TCLAP flags as the other offline CLIs in
 * this suite, plus an `--imu-topic` flag and an IMU sensor declared for the
 * bag readers, since this method is inertial.
 *
 * Every observation of every dataset entry is fed, in the order the dataset
 * holds them, so the inertial samples reach the estimator before the scans
 * whose segments they span. The estimator is synchronous, so there is nothing
 * to drain: a call returns only once the scan has been consumed.
 *
 * This path is deterministic. The trajectory is flushed with `finish()` before
 * being written, so the knots still inside the sliding window are not lost.
 */

#include <mola_ct_lio/CtLidarInertialOdometry.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#if defined(HAVE_MOLA_INPUT_MULRAN)
#include <mola_input_mulran_dataset/MulranDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
#include <mola_input_rawlog/RawlogDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
#include <mola_input_rosbag1/Rosbag1Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#endif

namespace
{

struct Cli
{
  TCLAP::CmdLine cmd{"mola-ct-lio-cli"};

  TCLAP::ValueArg<std::string> argYAML{
    "c",  "config", "Input Traj-LO pipeline YAML config file (required)",
    true, "",       "trajlo-oxford-spires.yaml",
    cmd};

  TCLAP::ValueArg<std::string> arg_verbosity_level{
    "v",    "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG {Default: INFO}", false, "",
    "INFO", cmd};

  TCLAP::ValueArg<std::string> arg_plugins{
    "l",   "load-plugins", "One or more {comma separated} *.so files to load as plugins",
    false, "foobar.so",    "foobar.so",
    cmd};

  TCLAP::ValueArg<std::string> arg_outPath{
    "",
    "output-tum-path",
    "Save the estimated path as a TXT file using the TUM file format (see evo docs)",
    false,
    "output-trajectory.txt",
    "output-trajectory.txt",
    cmd};

  TCLAP::ValueArg<int> arg_firstN{
    "",
    "only-first-n",
    "Run for the first N steps only (0=default, not used)",
    false,
    0,
    "Number of dataset entries to run",
    cmd};

#if defined(HAVE_MOLA_INPUT_MULRAN)
  TCLAP::ValueArg<std::string> argMulranSeq{
    "",    "input-mulran-seq", "INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...",
    false, "KAIST01",          "KAIST01",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  TCLAP::ValueArg<std::string> argRawlog{
    "",    "input-rawlog",   "INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)",
    false, "dataset.rawlog", "dataset.rawlog",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  TCLAP::ValueArg<std::string> argRosbag2{
    "",    "input-rosbag2", "INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)",
    false, "dataset.mcap",  "dataset.mcap",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  TCLAP::ValueArg<std::string> argRosbag1{
    "",    "input-rosbag1", "INPUT DATASET: rosbag1. Input dataset in ROS 1 bag format (*.bag)",
    false, "dataset.bag",   "dataset.bag",
    cmd};
#endif

// Unlike upstream Traj-LO, this estimator is inertial, so the bag readers
// below declare an IMU, and optionally an external odometry where a dataset
// publishes one.
#if defined(HAVE_MOLA_INPUT_ROSBAG2) || defined(HAVE_MOLA_INPUT_ROSBAG1)
  TCLAP::ValueArg<std::string> arg_imuTopic{
    "",          "imu-topic", "IMU topic in the rosbag (default: /imu/data)", false, "/imu/data",
    "/imu/data", cmd};

  TCLAP::ValueArg<std::string> arg_lidarTopic{
    "",    "lidar-topic", "Only for rosbag1/rosbag2 input: the LiDAR point cloud topic name.",
    false, "/lidar",      "/lidar",
    cmd};

  TCLAP::ValueArg<std::string> arg_odometryTopic{
    "",
    "odometry-topic",
    "External odometry topic, e.g. a legged platform's own kinematic-inertial "
    "estimate. Empty (the default) declares no such sensor.",
    false,
    "",
    "/odom",
    cmd};
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
  TCLAP::ValueArg<std::string> argKittiSeq{
    "",
    "input-kitti-seq",
    "INPUT DATASET: Use KITTI dataset sequence number 00|01|...",
    false,
    "00",
    "00",
    cmd};
#endif
};  // end struct "Cli"

#if defined(HAVE_MOLA_INPUT_MULRAN)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_mulran(
  const std::string & mulranSequence, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::MulranDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${MULRAN_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_ground_truth: true
)"""",
    mulranSequence.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rawlog(
  const std::string & rawlogFile, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::RawlogDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rawlog_filename: '%s'
      read_all_first: true
)"""",
    rawlogFile.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2) || defined(HAVE_MOLA_INPUT_ROSBAG1)
/** A comma-separated value becomes a YAML sequence, so that a recording split
 * across several bags (e.g. Oxford Spires keble-college-04, or GrandTour's
 * per-topic bags) is replayed as the single sequence it is. Both
 * Rosbag1Dataset and Rosbag2Dataset accept either a scalar or a sequence.
 */
std::string bags_to_yaml(const std::string & commaSeparated)
{
  std::vector<std::string> parts;
  mrpt::system::tokenize(commaSeparated, ",", parts);
  ASSERT_(!parts.empty());

  if (parts.size() == 1) {
    return "'" + mrpt::system::trim(parts[0]) + "'";
  }

  std::string out;
  for (const auto & p : parts) {
    out += "\n        - '" + mrpt::system::trim(p) + "'";
  }
  return out;
}

/** The odometry sensor entry for the bag reader, or nothing when no topic was
 * asked for. Read as CObservationRobotPose rather than the planar
 * CObservationOdometry: a legged platform's estimate carries height, roll and
 * pitch, which the planar type would silently drop.
 */
std::string odometrySensorYaml(const std::string & topic)
{
  if (topic.empty()) {
    return {};
  }
  return mrpt::format(
    "        - topic: '%s'\n"
    "          type: CObservationRobotPose\n"
    "          sensorLabel: odometry\n",
    topic.c_str());
}

#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag2(
  Cli & cli, const std::string & rosbag2file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag2Dataset>();
  o->setMinLoggingLevel(logLevel);

  // Fixed sensor poses (env vars), for bags with no /tf or /tf_static (e.g.
  // Oxford Spires): same env var names as mola-lidar-odometry-cli's own
  // dataset_from_rosbag2(), so the same override snippet works for every
  // wrapper in this benchmark suite.
  //
  // The /tf topic names are hooks too, since a bag recorded under a namespace
  // carries them as e.g. `/robot1/tf`, and a reader looking at `/tf` then finds
  // no extrinsics at all rather than failing.
  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: "${MOLA_TF_BASE_LINK|base_link}"
      tf_topic: "${MOLA_TF_TOPIC|/tf}"
      tf_static_topic: "${MOLA_TF_STATIC_TOPIC|/tf_static}"
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          sensorLabel: lidar
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          sensorLabel: imu
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
%s)"""",
    bags_to_yaml(rosbag2file).c_str(), cli.arg_lidarTopic.getValue().c_str(),
    cli.arg_imuTopic.getValue().c_str(),
    odometrySensorYaml(cli.arg_odometryTopic.getValue()).c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag1(
  Cli & cli, const std::string & rosbag1file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag1Dataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: "${MOLA_TF_BASE_LINK|base_link}"
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          sensorLabel: lidar
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          sensorLabel: imu
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
%s)"""",
    bags_to_yaml(rosbag1file).c_str(), cli.arg_lidarTopic.getValue().c_str(),
    cli.arg_imuTopic.getValue().c_str(),
    odometrySensorYaml(cli.arg_odometryTopic.getValue()).c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
/** KITTI needs no wrapping here, unlike in `mola-dlio-cli`: that CLI has to
 * interleave a synthetic IMU stream because DLIO cannot run without one, while
 * Traj-LO is LiDAR-only and takes the sequence as it is.
 *
 * KITTI's clouds carry no per-point time field, so the adapter's azimuth
 * fallback runs (it says so at INFO on the first scan). See
 * pipelines/trajlo-kitti.yaml for the consequences.
 */
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_kitti(
  const std::string & kittiSeqNumber, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::KittiOdometryDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      clouds_as_organized_points: false
      publish_lidar: true
      publish_image_0: false
      publish_image_1: false
      publish_ground_truth: true
)"""",
    kittiSeqNumber.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

void mola_signal_handler(int s)
{
  std::cerr << "Caught signal " << s << ". Shutting down...\n";
  exit(0);  // NOLINT
}

void mola_install_signal_handler()
{
  struct sigaction sigIntHandler
  {
  };
  sigIntHandler.sa_handler = &mola_signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, nullptr);
}

int main_odometry(Cli & cli)
{
  auto lio = mola::CtLidarInertialOdometry::Create();

  mrpt::system::VerbosityLevel logLevel = lio->getMinLoggingLevel();
  if (cli.arg_verbosity_level.isSet()) {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    logLevel = vl::name2value(cli.arg_verbosity_level.getValue());
    lio->setVerbosityLevel(logLevel);
  }

  // Initialize Traj-LO (no 'raw_data_source': we feed it directly below):
  const auto cfg = mola::load_yaml_file(cli.argYAML.getValue());
  lio->initialize(cfg);

  // Select dataset input:
  std::shared_ptr<mola::OfflineDatasetSource> dataset;

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  if (cli.argRawlog.isSet()) {
    dataset = dataset_from_rawlog(cli.argRawlog.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_MULRAN)
    if (cli.argMulranSeq.isSet()) {
    dataset = dataset_from_mulran(cli.argMulranSeq.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    if (cli.argRosbag2.isSet()) {
    dataset = dataset_from_rosbag2(cli, cli.argRosbag2.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)

    if (cli.argRosbag1.isSet()) {
    dataset = dataset_from_rosbag1(cli, cli.argRosbag1.getValue(), logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
    if (cli.argKittiSeq.isSet()) {
    dataset = dataset_from_kitti(cli.argKittiSeq.getValue(), logLevel);
  } else
#endif
  {
    THROW_EXCEPTION("At least one of the dataset input CLI flags must be defined. Use --help.");
  }
  ASSERT_(dataset);

  // Save GT, if available:
  if (cli.arg_outPath.isSet() && dataset->hasGroundTruthTrajectory()) {
    using namespace std::string_literals;
    const auto gtPath = dataset->getGroundTruthTrajectory();
    const auto gtOutFile = mrpt::system::fileNameChangeExtension(cli.arg_outPath.getValue(), "") +
                           "_gt."s + mrpt::system::extractFileExtension(cli.arg_outPath.getValue());
    std::cout << "Ground truth available. Saving it to: " << gtOutFile << "\n";
    gtPath.saveToTextFile_TUM(gtOutFile);
  }

  const double tStart = mrpt::Clock::nowDouble();

  size_t lastDatasetEntry = dataset->datasetSize();
  if (cli.arg_firstN.isSet()) {
    lastDatasetEntry = static_cast<size_t>(cli.arg_firstN.getValue());
  }
  mrpt::keep_min(lastDatasetEntry, dataset->datasetSize());

  size_t nLidarFed = 0;

  std::cout << "\n";  // Needed for the VT100 codes below.

  for (size_t i = 0; i < lastDatasetEntry; i++) {
    const auto sf = dataset->datasetGetObservations(i);
    ASSERT_(sf);

    // Everything is fed, IMU included: this is a LiDAR-inertial method, and
    // the inertial samples have to reach the estimator before the scans whose
    // segments they span.
    for (size_t k = 0; k < sf->size(); k++) {
      const auto obs = sf->getObservationByIndex(k);
      if (!obs) {
        continue;
      }
      lio->onNewObservation(obs);
    }

    nLidarFed = lio->scansProcessed();

    static int cnt = 0;
    if (cnt++ % 100 == 0) {
      cnt = 0;
      const size_t N = (dataset->datasetSize() - 1);
      const double pc = N > 0 ? static_cast<double>(i) / static_cast<double>(N) : 1.0;
      const double tNow = mrpt::Clock::nowDouble();
      const double ETA = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
      const double totalTime = ETA + (tNow - tStart);

      std::cout << "\033[A\33[2KT\r" << mrpt::system::progress(pc, 30)
                << mrpt::format(
                     " %6zu/%6zu (%.02f%%) ETA=%s/T=%s | scans fed=%zu\n", i, N, 100 * pc,
                     mrpt::system::formatTimeInterval(ETA).c_str(),
                     mrpt::system::formatTimeInterval(totalTime).c_str(), nLidarFed);
      std::cout.flush();
    }
  }

  // Flush the sliding window, so its last knots make it into the trajectory too.
  lio->finish();

  const double wallClock = mrpt::Clock::nowDouble() - tStart;
  const auto path = lio->estimatedTrajectory();

  std::cout << "\nDone. Dataset entries processed: " << lastDatasetEntry
            << ", LiDAR scans fed: " << nLidarFed << ", control poses estimated: " << path.size()
            << ", wall-clock: " << mrpt::system::formatTimeInterval(wallClock) << "\n";

  if (cli.arg_outPath.isSet()) {
    const auto fil = cli.arg_outPath.getValue();
    std::cout << "Saving estimated path in TUM format to: " << fil << "\n";
    path.saveToTextFile_TUM(fil);
  }

  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;
    if (!cli.cmd.parse(argc, argv)) {
      return 1;
    }

    if (cli.arg_plugins.isSet()) {
      std::string errMsg;
      const auto plugins = cli.arg_plugins.getValue();
      std::cout << "Loading plugin(s): " << plugins << "\n";
      if (!mrpt::system::loadPluginModules(plugins, errMsg)) {
        std::cerr << errMsg << std::endl;
        return 1;
      }
    }

    mola_install_signal_handler();
    return main_odometry(cli);
  } catch (std::exception & e) {
    mola::pretty_print_exception(e, "Exit due to exception:");
    return 1;
  }
}
