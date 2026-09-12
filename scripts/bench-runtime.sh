#!/bin/bash
# Wall-clock cost per scan, with and without the map's background rebuild.
#
# Map insertion, not the optimizer, is what this estimator spends its time on:
# profiled on a 2893-scan sequence, mapInsert took 1200 s of a 1957 s total,
# against 523 s for the whole sliding-window solve. The tree rebuild inside it
# runs synchronously by default so that two runs agree bit for bit, and this
# script measures what that costs.
#
# Run it on an otherwise idle machine: the two arms are compared against each
# other, so anything else competing for cores invalidates both.
#
# Usage:
#   bench-runtime.sh <mission-dir> [cores]
# e.g.
#   bench-runtime.sh /data/grand-tour/2024-11-02-17-18-32 0-15
set -e

DIR=${1:?usage: bench-runtime.sh <mission-dir> [cores]}
CORES=${2:-0-15}
M=$(basename "$DIR")
OUT=${OUT:-$PWD/ctlio-bench}
mkdir -p "$OUT"

CLI=$(command -v mola-ct-lio-cli || echo "$HOME/ros2_ws/install/mola_ct_lio/lib/mola_ct_lio/mola-ct-lio-cli")
YAML=${YAML:-$HOME/ros2_ws/src/mola_ct_lio/pipelines/ctlio-grandtour.yaml}

echo "mission : $M"
echo "cores   : $CORES  ($(nproc) on this machine)"
echo "binary  : $CLI"
echo

run () {   # tag  async
  echo "--- map_async_rebuild=$2 ---"
  ( export MOLA_TF_BASE_LINK=base CTLIO_PROFILE=true CTLIO_MAP_ASYNC=$2
    /usr/bin/time -f "  wall %e s   peak-rss %M kB" \
    taskset -c "$CORES" "$CLI" -c "$YAML" \
      --input-rosbag1 "$DIR/${M}_hesai_undist.bag,$DIR/${M}_tf_minimal.bag,$DIR/${M}_adis.bag" \
      --lidar-topic /boxi/hesai/points_undistorted --imu-topic /boxi/adis/imu \
      -v INFO --output-tum-path "$OUT/$1.tum" ) > "$OUT/$1.log" 2>&1 || true

  grep -E "^\s*wall|peak-rss" "$OUT/$1.log" | tail -1
  grep -E "Done\. Dataset" "$OUT/$1.log" | tail -1 | sed 's/^/  /'
  echo "  profile (mean per call):"
  grep -E "^(mapInsert|\+-> \.prune|optimizeWindow|\+-> \.match|match\.nnSearchCov2Cov|buildSegment)" \
    "$OUT/$1.log" | tail -8 | sed 's/^/    /'
  echo
}

run sync  false
run async true

echo "=== do the two agree? (they should NOT, if async actually engaged) ==="
if cmp -s "$OUT/sync.tum" "$OUT/async.tum"; then
  echo "  identical - async made no difference, check the build has the option"
else
  echo "  differ, as expected: a rebuild in flight changes query results"
  python3 - "$OUT/sync.tum" "$OUT/async.tum" <<'PY' || true
import sys
import numpy as np
a = np.loadtxt(sys.argv[1])
b = np.loadtxt(sys.argv[2])
n = min(len(a), len(b))
d = np.linalg.norm(a[:n, 1:4] - b[:n, 1:4], axis=1)
print("  trajectory difference: median %.4g m, max %.4g m, %d vs %d poses"
      % (np.median(d), d.max(), len(a), len(b)))
PY
fi
echo
echo "Report back: the two wall times, the two mapInsert means, and the"
echo "trajectory difference. Everything is under $OUT."
