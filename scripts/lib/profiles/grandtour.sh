# GrandTour (https://grand-tour.leggedrobotics.com): an ANYbotics ANYmal-D
# quadruped carrying the open-source "Boxi" sensor payload.
#
# Each mission is published as a set of per-topic ROS 1 bags, so the LiDAR, the
# IMU, /tf and the front camera live in SEPARATE files. They are replayed
# jointly, which is why both arms take a mission directory rather than a bag.
#
# Bags used (the rest of each mission's bags are ignored):
#   <mission>_hesai_undist.bag           /boxi/hesai/points_undistorted      10 Hz  (default LiDAR)
#   <mission>_livox_undist.bag           /boxi/livox/points_undistorted      10 Hz  (opt-in alternative LiDAR)
#   <mission>_anymal_velodyne_undist.bag /anymal/velodyne/points_undistorted 10 Hz  (opt-in alternative LiDAR)
#   <mission>_tf_minimal.bag    /tf, /tf_static                          (required, see below)
#   <mission>_tf_model.bag      /tf, /tf_static                          (only for the STIM320)
#   <mission>_adis.bag          /boxi/adis/imu                   200 Hz  (default IMU)
#   <mission>_stim320_imu.bag   /boxi/stim320/imu                500 Hz  (opt-in alternative IMU)
#   <mission>_hdr_<tag>.bag     /boxi/hdr/<tag>/image_raw/...    10 Hz   (optional, GUI only)
#   <mission>_alphasense.bag    /boxi/alphasense/<tag>/image_...  10 Hz   (optional, GUI only)
#   <mission>_anymal_state.bag  /anymal/state_estimator/odometry         (opt-in, see below)
#
# Extrinsics come from the dataset's own /tf_static (base -> box_base ->
# hesai_lidar / adis16475_imu; base -> velodyne_lidar), which is why the
# pipeline leaves baselink2lidar_pose_str at identity. The TF bag is therefore
# not optional here the way it is for a LiDAR-only method: an inertial
# estimator with the IMU placed at the vehicle origin is not a degraded run,
# it is a wrong one.
#
# The published Hesai/Livox/Velodyne streams are all "_undist", i.e. already
# motion compensated onto one instant. That is what ctlio-grandtour.yaml's
# `clouds_already_deskewed` is about; see its comment before changing the
# LiDAR selection.
#
# Scoring against this dataset's ground truth needs a body-frame correction:
# every GT source is anchored to a physical sensor mount (cpt7_imu, 0.293 m
# from base; or the total-station prism, 0.395 m), not to base, which is what
# the odometry reports. scripts/score-grandtour.py does that.

mola_ctlio_profile_usage() {
  echo "Error: A GrandTour mission directory (or its '*_hesai_undist.bag') is required."
  echo "Usage: $0 /path/to/<mission-dir>/ [additional flags]"
  echo "       $0 /path/to/<mission>_hesai_undist.bag [additional flags]"
  echo ""
  echo "Optional environment variables:"
  echo "  MOLA_GRANDTOUR_LIDAR   which LiDAR to use: 'hesai' (default, Boxi payload),"
  echo "                         'livox' (Boxi payload) or 'velodyne' (mounted on the"
  echo "                         ANYmal body itself, not the payload)"
  echo "  MOLA_GRANDTOUR_IMU     which IMU to use: 'adis' (default, 200 Hz) or"
  echo "                         'stim320' (500 Hz, tactical grade). The STIM320 also"
  echo "                         requires <mission>_tf_model.bag, since its frame is"
  echo "                         absent from the smaller tf_minimal.bag"
  echo "  MOLA_ODOMETRY_TOPIC    the robot's legged kinematic-inertial odometry. Empty"
  echo "                         (the default) declares no such sensor; setting it adds"
  echo "                         <mission>_anymal_state.bag to the inputs. The pipeline"
  echo "                         must ALSO be told to believe it, with nonzero"
  echo "                         CTLIO_ODO_SIGMA_LIN / CTLIO_ODO_SIGMA_ANG"
  echo "  MOLA_GRANDTOUR_CAMERA  which camera to preview in the GUI (ignored in CLI"
  echo "                         mode): 'hdr_front' (default), 'hdr_left', 'hdr_right',"
  echo "                         or one of the five Alphasense cameras"
  echo "                         'alphasense_front_center', 'alphasense_front_left',"
  echo "                         'alphasense_front_right', 'alphasense_left',"
  echo "                         'alphasense_right'"
  echo "  CTLIO_OUTPUT_TUM       where the offline arm writes the trajectory"
  echo "                         (default: ./ctlio-grandtour.tum; empty disables it)"
  echo ""
  echo "Every CTLIO_* pipeline knob works here too, since the pipeline reads them"
  echo "from the environment. See the repository's agents.md for the full list."
  echo ""
  echo "Example:"
  echo "  $0 ~/datasets/grand-tour/2024-10-01-11-29-55/"
}

# Map a LiDAR tag to the bag suffix and topic it is carried on.
_grandtour_lidar_spec() {
  case "$1" in
    hesai)
      _gt_lidar_suffix=_hesai_undist.bag
      _gt_lidar_topic=/boxi/hesai/points_undistorted
      ;;
    livox)
      _gt_lidar_suffix=_livox_undist.bag
      _gt_lidar_topic=/boxi/livox/points_undistorted
      ;;
    velodyne)
      _gt_lidar_suffix=_anymal_velodyne_undist.bag
      _gt_lidar_topic=/anymal/velodyne/points_undistorted
      ;;
    *)
      echo "Error: MOLA_GRANDTOUR_LIDAR must be 'hesai', 'livox' or 'velodyne', got '$1'." >&2
      return 1
      ;;
  esac
}

mola_ctlio_profile_resolve() {
  local arg=$1
  shift
  CTLIO_EXTRA_ARGS=("$@")

  : "${MOLA_GRANDTOUR_LIDAR:=hesai}"
  _grandtour_lidar_spec "$MOLA_GRANDTOUR_LIDAR" || return 1
  local lidar_suffix=$_gt_lidar_suffix
  local lidar_topic=$_gt_lidar_topic

  # Resolve the "<dir>/<mission>" prefix shared by all of a mission's bags,
  # from either a mission directory or any one of its bag files:
  local lidar_bag
  if [ -d "$arg" ]; then
    lidar_bag=$(ls "${arg%/}"/*"$lidar_suffix" 2>/dev/null | head -n 1)
    if [ -z "$lidar_bag" ]; then
      echo "Error: no '*$lidar_suffix' found in directory '$arg' (MOLA_GRANDTOUR_LIDAR=$MOLA_GRANDTOUR_LIDAR)." >&2
      return 1
    fi
  elif [ -f "$arg" ]; then
    lidar_bag=$arg
  else
    echo "Error: '$arg' is neither a directory nor a file." >&2
    return 1
  fi

  local prefix=${lidar_bag%"$lidar_suffix"}
  if [ "$prefix" = "$lidar_bag" ]; then
    echo "Error: '$lidar_bag' does not follow the expected '<mission>$lidar_suffix' naming." >&2
    return 1
  fi

  # Which IMU: the ADIS16475 by default, or the higher-grade STIM320.
  #
  # The two are not interchangeable as far as /tf goes. `tf_minimal.bag`
  # publishes the ADIS's frame but NOT the STIM320's, so selecting the STIM320
  # also selects the full `tf_model.bag`, which carries every sensor frame.
  : "${MOLA_GRANDTOUR_IMU:=adis}"
  local tf_bag=${prefix}_tf_minimal.bag
  local imu_bag=${prefix}_adis.bag
  local imu_topic=/boxi/adis/imu

  case "$MOLA_GRANDTOUR_IMU" in
    adis) ;;
    stim320)
      tf_bag=${prefix}_tf_model.bag
      imu_bag=${prefix}_stim320_imu.bag
      imu_topic=/boxi/stim320/imu
      ;;
    *)
      echo "Error: MOLA_GRANDTOUR_IMU must be 'adis' or 'stim320', got '$MOLA_GRANDTOUR_IMU'." >&2
      return 1
      ;;
  esac

  # A TF bag with refined extrinsics can be supplied in place of whichever
  # recorded one the IMU choice selected. Applied last, so a caller that sets
  # both variables gets the override rather than the STIM320 default.
  tf_bag=${MOLA_GRANDTOUR_TF_BAG:-$tf_bag}

  echo "GrandTour mission '$(basename "$prefix")':"
  echo "  LiDAR bag: $lidar_bag  ($MOLA_GRANDTOUR_LIDAR)"

  local -a bags=("$lidar_bag")

  # Not optional, unlike in a LiDAR-only wrapper: without /tf the IMU lands at
  # the vehicle origin, and an inertial estimator given the wrong lever arm
  # produces a plausible trajectory that is simply wrong.
  if [ ! -f "$tf_bag" ]; then
    echo "Error: the extrinsics bag '$tf_bag' was not found." >&2
    echo "       This estimator is inertial: without /tf the IMU would be placed at" >&2
    echo "       the vehicle origin, which is a wrong run rather than a degraded one." >&2
    echo "       Fetch it with: klein download -p GrandTourDataset -m release_<mission> \\" >&2
    echo "                        --dest <dir> --create-dirs -y <mission>_tf_minimal.bag" >&2
    return 1
  fi
  echo "  TF bag   : $tf_bag"
  bags+=("$tf_bag")

  if [ ! -f "$imu_bag" ]; then
    echo "Error: the IMU bag '$imu_bag' was not found (MOLA_GRANDTOUR_IMU=$MOLA_GRANDTOUR_IMU)." >&2
    echo "       Run LiDAR-only with CTLIO_USE_IMU=false if that is what you meant." >&2
    return 1
  fi
  echo "  IMU bag  : $imu_bag  ($MOLA_GRANDTOUR_IMU)"
  bags+=("$imu_bag")

  # External odometry: OFF by default, matching the pipeline. The factor is
  # measured and useful, but a second pose source fused while the
  # LiDAR-inertial core still has unexplained failures only makes those harder
  # to read. Note "=" and not ":=": an explicitly empty value is how a caller
  # turns it off, and ":=" would treat that as unset.
  : "${MOLA_ODOMETRY_TOPIC=}"
  export MOLA_ODOMETRY_TOPIC
  if [ -n "$MOLA_ODOMETRY_TOPIC" ]; then
    local odom_bag=${prefix}_anymal_state.bag
    if [ -f "$odom_bag" ]; then
      echo "  Odometry bag: $odom_bag"
      bags+=("$odom_bag")
    else
      echo "  Odometry bag: (not found: '$odom_bag'; MOLA_ODOMETRY_TOPIC matches nothing)"
    fi
    if [ "${CTLIO_ODO_SIGMA_LIN:-0.0}" = "0.0" ] && [ "${CTLIO_ODO_SIGMA_ANG:-0.0}" = "0.0" ]; then
      echo "  Note: the odometry factor is disabled in the pipeline. Set"
      echo "        CTLIO_ODO_SIGMA_LIN / CTLIO_ODO_SIGMA_ANG for it to be used."
    fi
  fi

  # The camera is a GUI preview only: nothing in this estimator consumes it, so
  # a batch run would pay for decoding several GB of JPEG for nothing. The
  # three HDR cameras each ship in their own bag while the five Alphasense
  # cameras share one, so the bag and the topic are resolved together.
  if [ "$CTLIO_MODE" = "gui" ]; then
    : "${MOLA_GRANDTOUR_CAMERA:=hdr_front}"
    local camera_bag
    local camera_topic

    case "$MOLA_GRANDTOUR_CAMERA" in
      hdr_front | hdr_left | hdr_right)
        camera_bag=${prefix}_${MOLA_GRANDTOUR_CAMERA}.bag
        camera_topic=/boxi/hdr/${MOLA_GRANDTOUR_CAMERA#hdr_}/image_raw/compressed
        ;;
      alphasense_front_center | alphasense_front_left | alphasense_front_right | \
        alphasense_left | alphasense_right)
        camera_bag=${prefix}_alphasense.bag
        camera_topic=/boxi/alphasense/${MOLA_GRANDTOUR_CAMERA#alphasense_}/image_raw/compressed
        ;;
      *)
        echo "Error: MOLA_GRANDTOUR_CAMERA must be one of hdr_{front,left,right} or" >&2
        echo "       alphasense_{front_center,front_left,front_right,left,right}," >&2
        echo "       got '$MOLA_GRANDTOUR_CAMERA'." >&2
        return 1
        ;;
    esac

    if [ -f "$camera_bag" ]; then
      echo "  Camera bag: $camera_bag  ($MOLA_GRANDTOUR_CAMERA)"
      bags+=("$camera_bag")
      : "${MOLA_CAMERA_TOPIC:=$camera_topic}"
      export MOLA_CAMERA_TOPIC
    else
      echo "  Camera bag: (not found: '$camera_bag'; no camera preview)"
    fi
  fi

  # Topics and frames. The offline CLI reads the frame from the environment and
  # takes the topics as flags; the library turns these same variables into the
  # flags, so they are stated once here either way.
  : "${MOLA_LIDAR_TOPIC:=$lidar_topic}"
  : "${MOLA_IMU_TOPIC:=$imu_topic}"
  : "${MOLA_TF_BASE_LINK:=base}"  # the ANYmal body frame; NOT named 'base_link'
  export MOLA_LIDAR_TOPIC MOLA_IMU_TOPIC MOLA_TF_BASE_LINK

  mola_ctlio_bag_slots "${bags[@]}" || return 1

  CTLIO_LAUNCH_FILE=ct_lio_from_rosbag1.yaml
  CTLIO_PIPELINE=ctlio-grandtour.yaml
  CTLIO_CLI_INPUT=(--input-rosbag1 "$CTLIO_BAGS_JOINED")
}
