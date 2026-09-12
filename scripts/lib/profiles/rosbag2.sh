# Generic ROS 2 bag input: not a particular dataset, so this profile routes the
# bag and leaves every topic, frame and knob at its default, to be set from the
# environment. It is the one to use on your own recording.
#
# What this estimator needs that a LiDAR-only one does not: the IMU's pose in
# the body frame. It comes from the bag's /tf_static like any other extrinsic,
# so a bag carrying /tf needs nothing here. A bag without one needs
# MOLA_USE_FIXED_IMU_POSE=true and the IMU_POSE_* values, and getting them
# wrong does not fail the run, it quietly degrades the trajectory. See the
# Oxford Spires section of agents.md for a worked example, including how the
# body-frame conversion was checked against ground truth rather than assumed.

mola_ctlio_profile_usage() {
  echo "Error: A ROS 2 bag is required."
  echo "Usage: $0 /path/to/dataset.mcap [additional flags]"
  echo "       $0 /path/to/dataset_directory    (must contain a metadata.yaml)"
  echo ""
  echo "Topics and frames:"
  echo "  MOLA_LIDAR_TOPIC       LiDAR point cloud topic (default: '/ouster/points')"
  echo "  MOLA_IMU_TOPIC         IMU topic (default: '/imu'). Set it EMPTY for a bag"
  echo "                         with no IMU, which selects the LiDAR-only arm"
  echo "  MOLA_ODOMETRY_TOPIC    external odometry (optional). The pipeline must ALSO"
  echo "                         be told to believe it, with nonzero"
  echo "                         CTLIO_ODO_SIGMA_LIN / CTLIO_ODO_SIGMA_ANG"
  echo "  MOLA_TF_BASE_LINK      robot base /tf frame id (default: 'base_link')"
  echo "  MOLA_TF_TOPIC          /tf topic in the bag (default: '/tf')"
  echo "  MOLA_TF_STATIC_TOPIC   /tf_static topic in the bag (default: '/tf_static')"
  echo "                         Override both for a namespaced bag, e.g. /robot1/tf."
  echo ""
  echo "For a bag with no /tf at all, the sensor poses have to be given instead:"
  echo "  MOLA_USE_FIXED_LIDAR_POSE=true  LIDAR_POSE_{X,Y,Z,YAW,PITCH,ROLL}"
  echo "  MOLA_USE_FIXED_IMU_POSE=true    IMU_POSE_{X,Y,Z,YAW,PITCH,ROLL}"
  echo "                         Angles in degrees, and the IMU pose is in the BODY"
  echo "                         frame, not the LiDAR's. A wrong lever arm degrades"
  echo "                         the trajectory silently rather than failing."
  echo ""
  echo "Pipeline and output:"
  echo "  CTLIO_PIPELINE_YAML    pipeline to run (default: the dataset-neutral"
  echo "                         pipelines/ctlio-generic.yaml)"
  echo "  CTLIO_OUTPUT_TUM       where the offline arm writes the trajectory"
  echo "                         (default: ./ctlio-rosbag2.tum; empty disables it)"
  echo ""
  echo "Every CTLIO_* pipeline knob works here too, since the pipeline reads them"
  echo "from the environment. See the repository's agents.md for the full list."
  echo ""
  echo "Example:"
  echo "  MOLA_LIDAR_TOPIC=/velodyne_points MOLA_IMU_TOPIC=/imu/data \\"
  echo "  MOLA_TF_BASE_LINK=base_footprint \\"
  echo "    $0 ~/bags/my-recording/"
}

mola_ctlio_profile_resolve() {
  local file=$1
  shift
  CTLIO_EXTRA_ARGS=("$@")

  if [ ! -e "$file" ]; then
    echo "Error: '$file' does not exist." >&2
    return 1
  fi

  MOLA_INPUT_ROSBAG2="$file"
  export MOLA_INPUT_ROSBAG2

  echo "ROS 2 bag: $file"

  # Stated once, then used as launch-file variables online and turned into
  # flags for the offline binary. Only the defaults the launch file already
  # documents are repeated here, because the CLI takes them as flags and so
  # cannot fall back on its own.
  : "${MOLA_LIDAR_TOPIC:=/ouster/points}"
  # Note "=" and not ":=" on these two: an explicitly empty value is how a
  # caller declares the sensor absent, and ":=" would treat that as unset and
  # put the default back.
  : "${MOLA_IMU_TOPIC=/imu}"
  : "${MOLA_ODOMETRY_TOPIC=}"
  : "${MOLA_TF_BASE_LINK:=base_link}"
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC MOLA_ODOMETRY_TOPIC MOLA_TF_BASE_LINK

  # No inertial stream means the LiDAR-only arm, where the twist-continuity
  # term stands in for the IMU factors. Still overridable, since a bag can
  # carry an IMU the caller has decided not to name.
  if [ -z "$MOLA_IMU_TOPIC" ]; then
    : "${CTLIO_USE_IMU:=false}"
    export CTLIO_USE_IMU
  fi

  echo "  LiDAR topic: $MOLA_LIDAR_TOPIC"
  echo "  IMU topic  : ${MOLA_IMU_TOPIC:-(none, running LiDAR-only)}"
  if [ -n "$MOLA_ODOMETRY_TOPIC" ]; then
    echo "  Odom topic : $MOLA_ODOMETRY_TOPIC"
    if [ "${CTLIO_ODO_SIGMA_LIN:-0.0}" = "0.0" ] && [ "${CTLIO_ODO_SIGMA_ANG:-0.0}" = "0.0" ]; then
      echo "  Note: the odometry factor is disabled in the pipeline. Set"
      echo "        CTLIO_ODO_SIGMA_LIN / CTLIO_ODO_SIGMA_ANG for it to be used."
    fi
  fi
  echo "  Base frame : $MOLA_TF_BASE_LINK  (tf: ${MOLA_TF_TOPIC:-/tf}, ${MOLA_TF_STATIC_TOPIC:-/tf_static})"

  CTLIO_LAUNCH_FILE=ct_lio_from_rosbag2.yaml
  CTLIO_PIPELINE=ctlio-generic.yaml
  CTLIO_CLI_INPUT=(--input-rosbag2 "$file")
}
