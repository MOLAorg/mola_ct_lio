# Shared plumbing for the mola-ct-lio-gui-* / mola-ct-lio-cli-* dataset
# wrappers. Sourced, never executed.
#
# A "dataset profile" (lib/profiles/<name>.sh) describes ONE public dataset:
# where its bags are, what its topics and frames are called, and which pipeline
# it wants. It exports that description and invokes nothing, so the same
# description feeds every consumer:
#   - mola-ct-lio-gui-<name>   online replay through mola-cli + a launch YAML,
#   - mola-ct-lio-cli-<name>   offline batch through mola-ct-lio-cli,
#   - out-of-tree harnesses (sweeps, scoring runs) that source a profile
#     directly instead of restating the dataset.
#
# This mirrors the mola_lidar_odometry wrappers deliberately, with one
# difference that matters: mola-ct-lio-cli takes its topics as command-line
# FLAGS rather than reading MOLA_* variables. So a profile publishes the
# topics as variables for the launch YAML, and mola_ctlio_exec_cli below turns
# the same variables into flags. Nothing states a topic twice.
#
# ---------------------------------------------------------------------------
# Profile contract
# ---------------------------------------------------------------------------
# A profile file defines two functions:
#
#   mola_ctlio_profile_usage     Prints the dataset-specific usage message.
#
#   mola_ctlio_profile_resolve   Receives the wrapper's arguments. Consumes the
#                                ones naming the dataset, exports the MOLA_*
#                                and CTLIO_* description, and sets:
#                                  CTLIO_LAUNCH_FILE  launch YAML basename
#                                  CTLIO_PIPELINE     pipeline YAML basename
#                                  CTLIO_CLI_INPUT    array of offline input
#                                                     flags, e.g.
#                                                     (--input-rosbag1 a,b)
#                                  CTLIO_EXTRA_ARGS   leftover args, forwarded
#                                                     verbatim
#
# One variable is set by the caller before the profile runs, and profiles may
# branch on it:
#
#   CTLIO_MODE                  "gui" or "cli". Some inputs are worth carrying
#                               in one and are dead weight in the other: a
#                               camera bag is a GUI preview and several GB of
#                               JPEG a batch run decodes for nothing.
#
# Every value a profile publishes uses the ': "${VAR:=default}"' idiom, so a
# caller that exported the variable first always wins. That is what lets a
# sweep keep its per-run overrides without forking the profile, and it is the
# same rule the pipeline YAMLs follow for their own CTLIO_* knobs.

# Absolute path of this library's directory, resolved through symlinks.
CTLIO_LIB_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

# Where mola-cli-launchs/ and pipelines/ live. This library sits in
# scripts/lib/ in a source checkout and in share/mola_ct_lio/scripts-lib/ once
# installed, so those data directories are one level up in the installed case
# and two in the source one.
mola_ctlio_share_dir() {
  if [ -d "$CTLIO_LIB_DIR/../mola-cli-launchs" ]; then
    (cd -- "$CTLIO_LIB_DIR/.." && pwd)
  else
    (cd -- "$CTLIO_LIB_DIR/../.." && pwd)
  fi
}

CTLIO_SHARE_DIR=$(mola_ctlio_share_dir)
CTLIO_LAUNCHS_DIR="$CTLIO_SHARE_DIR/mola-cli-launchs"
CTLIO_PIPELINES_DIR="$CTLIO_SHARE_DIR/pipelines"

# The offline binary installs to lib/mola_ct_lio/ rather than bin/, so it is
# not on PATH after a plain `source install/setup.bash`. Prefer whatever PATH
# offers (a development build, or a future move to bin/), then ask ament, then
# fall back to the conventional workspace layout.
mola_ctlio_cli_binary() {
  local found
  found=$(command -v mola-ct-lio-cli 2>/dev/null) && {
    echo "$found"
    return 0
  }

  local prefix
  prefix=$(ros2 pkg prefix mola_ct_lio 2>/dev/null)
  if [ -n "$prefix" ] && [ -x "$prefix/lib/mola_ct_lio/mola-ct-lio-cli" ]; then
    echo "$prefix/lib/mola_ct_lio/mola-ct-lio-cli"
    return 0
  fi

  echo "Error: 'mola-ct-lio-cli' was not found. Build the package and source" >&2
  echo "       the workspace's install/setup.bash first." >&2
  return 1
}

# Fills the five MOLA_INPUT_ROSBAG1* slots the launch file exposes from the bag
# list given as arguments, and sets CTLIO_BAGS_JOINED to the same list
# comma-joined, which is the spelling --input-rosbag1 takes. Empty arguments
# are dropped; unused slots are set to the empty string rather than left alone,
# so a value exported by an earlier run cannot leak in as an extra bag.
#
# More bags than slots is not an error: they all travel in the first slot as
# one comma-separated list, which Rosbag1Dataset splits exactly as the offline
# CLI does.
#
# The joined list is returned through a variable, not echoed: a caller
# capturing it with $(...) would run this in a subshell and lose the exports.
mola_ctlio_bag_slots() {
  local -a bags=()
  local b
  for b in "$@"; do
    [ -n "$b" ] && bags+=("$b")
  done

  local max_slots=5

  local joined=""
  for b in "${bags[@]}"; do
    joined="${joined:+$joined,}$b"
  done
  CTLIO_BAGS_JOINED="$joined"

  local i
  local var
  local value
  for ((i = 0; i < max_slots; i++)); do
    var="MOLA_INPUT_ROSBAG1"
    if [ "$i" -gt 0 ]; then
      var="MOLA_INPUT_ROSBAG1_$((i + 1))"
    fi

    if [ "${#bags[@]}" -le "$max_slots" ]; then
      value="${bags[$i]:-}"
    elif [ "$i" -eq 0 ]; then
      value="$joined"
    else
      value=""
    fi

    printf -v "$var" '%s' "$value"
    export "${var?}"
  done
}

# Sources a profile and runs it. $1 is the profile name, the rest are the
# wrapper's own arguments. Returns non-zero on failure rather than exiting:
# callers other than the wrappers need to report it their own way.
mola_ctlio_load_profile() {
  local name=$1
  shift

  # Used for the default output file name. Deliberately not CTLIO_PROFILE,
  # which the pipeline YAMLs already use for the profiler switch.
  CTLIO_PROFILE_NAME=$name

  local profile="$CTLIO_LIB_DIR/profiles/$name.sh"
  if [ ! -f "$profile" ]; then
    echo "Error: no dataset profile '$name' (looked in '$CTLIO_LIB_DIR/profiles/')." >&2
    return 1
  fi

  CTLIO_LAUNCH_FILE=""
  CTLIO_PIPELINE=""
  CTLIO_CLI_INPUT=()
  CTLIO_EXTRA_ARGS=()

  # shellcheck source=/dev/null
  source "$profile"

  if [ "$#" -eq 0 ]; then
    mola_ctlio_profile_usage
    return 1
  fi

  mola_ctlio_profile_resolve "$@"
}

# The pipeline YAML both arms run. A profile names one; CTLIO_PIPELINE_YAML
# overrides it with an absolute path, which is how a variant pipeline is tried
# without editing anything.
mola_ctlio_pipeline_yaml() {
  echo "${CTLIO_PIPELINE_YAML:-$CTLIO_PIPELINES_DIR/$CTLIO_PIPELINE}"
}

# Online: mola-cli + the profile's launch YAML. The module is found through
# LD_LIBRARY_PATH, so a sourced workspace is all it takes.
mola_ctlio_exec_gui() {
  local launch="$CTLIO_LAUNCHS_DIR/$CTLIO_LAUNCH_FILE"
  if [ ! -f "$launch" ]; then
    echo "Error: launch file not found: '$launch'" >&2
    exit 1
  fi

  CTLIO_PIPELINE_YAML=$(mola_ctlio_pipeline_yaml)
  export CTLIO_PIPELINE_YAML

  exec mola-cli "$launch" "${CTLIO_EXTRA_ARGS[@]}" "$@"
}

# Offline: mola-ct-lio-cli + the profile's input flags. Topics travel as flags
# because that is what this binary reads; the frame and the fixed-pose choices
# stay in the environment, which its bag reader does read.
mola_ctlio_exec_cli() {
  local cli
  cli=$(mola_ctlio_cli_binary) || exit 1

  local -a args=(-c "$(mola_ctlio_pipeline_yaml)")
  args+=("${CTLIO_CLI_INPUT[@]}")

  if [ -n "${MOLA_LIDAR_TOPIC:-}" ]; then
    args+=(--lidar-topic "$MOLA_LIDAR_TOPIC")
  fi
  if [ -n "${MOLA_IMU_TOPIC:-}" ]; then
    args+=(--imu-topic "$MOLA_IMU_TOPIC")
  fi
  if [ -n "${MOLA_ODOMETRY_TOPIC:-}" ]; then
    args+=(--odometry-topic "$MOLA_ODOMETRY_TOPIC")
  fi

  # Writing the trajectory is the point of a batch run, so it is on by default
  # here, unlike in the binary. CTLIO_OUTPUT_TUM= (empty) turns it off for a
  # run that only wants the log.
  : "${CTLIO_OUTPUT_TUM=$PWD/ctlio-$CTLIO_PROFILE_NAME.tum}"
  if [ -n "$CTLIO_OUTPUT_TUM" ]; then
    args+=(--output-tum-path "$CTLIO_OUTPUT_TUM")
    echo "  Output   : $CTLIO_OUTPUT_TUM"
  fi

  exec "$cli" "${args[@]}" "${CTLIO_EXTRA_ARGS[@]}" "$@"
}
