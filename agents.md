# agents.md - mola_ct_lio

Continuous-time LiDAR-inertial odometry. Traj-LO's trajectory mathematics,
`mp2p_icp` matching over an incremental point map, tightly-coupled IMU.

Design document: `~/plans/lio/detail/lio-ct-lio-design.md`.

## Layout

| path | what |
|---|---|
| `core/` | the estimator. Eigen only, no ROS/MRPT/MOLA, so it builds and tests standalone |
| `module/` | the MOLA front end: map, matchers, dataset input |
| `apps/` | offline CLI |
| `pipelines/`, `mola-cli-launchs/` | per-dataset configuration |

## Build and test

```bash
cd ~/ros2_ws && colcon build --packages-select mola_ct_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/mola_ct_lio/test_ct_jacobians && ./build/mola_ct_lio/test_window_optimizer
```

Every analytic Jacobian has a finite-difference test. Do not change one
without running it: an error there degrades the trajectory silently instead
of failing.

## Determinism

Required, and covered by `WindowOptimizer.IsBitwiseRepeatable`. The
correspondence order fixes the summation order of the assembly, so a matcher
that returns pairings in a thread-dependent order breaks it. Any parallel
reduction over points must be a deterministic one.

## Tuning parameters

Every knob, with its default and the range worth sweeping. Env-var overrides
follow the `${CTLIO_NAME|default}` pattern in the pipeline YAMLs. Keep this
table in sync when adding a parameter.

### Trajectory and window (`WindowOptimizer::Params`, estimator)

| parameter | env | default | sweep range | notes |
|---|---|---|---|---|
| `segment_interval` | `CTLIO_SEG_INTERVAL` | 0.04 s | 0.02 - 0.10 | knot spacing; upstream Traj-LO uses 0.04 |
| `knot_count` | `CTLIO_KNOT_COUNT` | 4 | 3 - 8 | window size; upstream uses 3 control poses |
| `init_interval` | `CTLIO_INIT_INTERVAL` | 0.3 s | 0.1 - 1.0 | data gathered before the first solve |
| `max_iterations` | `CTLIO_MAX_ITERS` | 25 | 5 - 40 | |
| `convergence_threshold` | `CTLIO_CONVERGE_TH` | 1e-3 m | 1e-4 - 1e-2 | on the largest knot translation step |
| `lambda` | `CTLIO_LAMBDA` | 0.0 | 0 - 1e-3 | Levenberg damping as a fraction of the diagonal; 0 is plain Gauss-Newton |
| `twist_continuity_weight` | `CTLIO_TWIST_W` | 2.0 | 0 - 10 | LiDAR-only only; ignored once IMU factors are present |

### Residual weighting

| parameter | env | default | sweep range | notes |
|---|---|---|---|---|
| `kernel` | `CTLIO_KERNEL` | `Cauchy` | None, Cauchy, GemanMcClure | Cauchy is upstream's form |
| `kernel_scale` | `CTLIO_KERNEL_SCALE` | 0.5 m | 0.1 - 2.0 | upstream ties this to the adaptive threshold over 3 |
| `rematch_every` | `CTLIO_REMATCH_EVERY` | 1 | 1 - 5 | iterations between correspondence searches. 1 is upstream's behavior and is only affordable with a cheap matcher; a GICP search over an incremental map wants more |

### IMU

| parameter | env | default | sweep range | notes |
|---|---|---|---|---|
| `use_imu` | `CTLIO_USE_IMU` | true | true/false | false is the LiDAR-only control arm |
| `gravity` | `CTLIO_GRAVITY` | (0,0,-9.81) | | world frame must be gravity aligned |
| `gyro_noise_density` | `CTLIO_GYRO_SIGMA` | 1.7e-4 | 1e-5 - 1e-3 | MOLA default; per-sensor in practice |
| `accel_noise_density` | `CTLIO_ACCEL_SIGMA` | **5.0e-2** | 2e-2 - 5e-1 | effective weight, not the datasheet figure; see the section above |
| `bias_sigma_acc` | `CTLIO_BIAS_SIGMA_ACC` | 1e-3 | 1e-5 - 1e-2 | bias random walk density |
| `bias_sigma_gyro` | `CTLIO_BIAS_SIGMA_GYRO` | 1e-4 | 1e-6 - 1e-3 | |
| `imu_init_seconds` | `CTLIO_IMU_INIT_SEC` | 1.0 | 0.5 - 3.0 | static window for initial attitude and bias |
| `max_initial_gyro_bias` | - | 0.02 rad/s | 0.005 - 0.05 | measured seeds beyond this are discarded as platform motion |
| `max_initial_accel_bias` | - | 0.5 m/s^2 | 0.1 - 1.0 | same, for the accelerometer |

### Point preprocessing

| parameter | env | default | sweep range | notes |
|---|---|---|---|---|
| `ds_size` | `CTLIO_DS_SIZE` | 0.4 m | 0.2 - 1.0 | per-segment voxel downsample of the source points |
| `min_range` | `CTLIO_MIN_RANGE` | 1.0 m | 0.3 - 3.0 | Oxford needs 1.0, KITTI 0.3 |
| `max_range` | `CTLIO_MAX_RANGE` | 100 m | 50 - 150 | |
| `decimation` | `CTLIO_DECIMATION` | 1 | 1 - 4 | the corpus has a standing result that decimation is free in both directions |

### Map (`mola::IncrementalPointCloud`)

| parameter | env | default | sweep range | notes |
|---|---|---|---|---|
| `voxel_size` | `CTLIO_VOXEL_SIZE` | 0.4 m | 0.2 - 1.0 | map insertion decimation |
| `remove_points_farther_than` | `CTLIO_MAP_RADIUS` | 120 m | 50 - 200 | sliding-window extent |
| `k_correspondences_for_cov` | `CTLIO_K_COV` | 20 | 8 - 30 | neighbors per GICP covariance; the corpus found 10 better on KITTI/Oxford for MOLA-LO |
| `min_correspondences_for_cov` | `CTLIO_MIN_K_COV` | 5 | 3 - 10 | below this an isotropic covariance is used |
| `max_distance_for_cov` | `CTLIO_MAX_DIST_COV` | 1.0 m | 0.5 - 3.0 | |
| `max_plane_deviation_for_cov` | `CTLIO_PLANE_DEV_COV` | 0 | 0 - 0.1 | 0 disables the plane regularization gate |

### Matcher (`mp2p_icp`)

| parameter | env | default | sweep range | notes |
|---|---|---|---|---|
| `matcher_class` | `CTLIO_MATCHER` | `Matcher_Cov2Cov` | Cov2Cov, Points_DistanceThreshold, Points_KnnPlane | the residual type is a configuration choice, see the design doc |
| `threshold` | `CTLIO_MATCH_TH` | 0.40 m | 0.2 - 1.5 | also the near value when `thresholdFar` is set |
| `threshold_far` | `CTLIO_MATCH_TH_FAR` | 0 | 0 - 2.0 | 0 keeps a flat threshold |
| `threshold_knee_range` | `CTLIO_MATCH_KNEE` | 15 m | 10 - 40 | only with `threshold_far` |
| `threshold_transition_width` | `CTLIO_MATCH_WIDTH` | 5 m | 2 - 15 | only with `threshold_far` |
| `adaptive_initial_threshold` | `CTLIO_ADAPT_TH0` | 1.5 m | 0.5 - 3.0 | KISS-ICP style controller |
| `adaptive_min_motion` | `CTLIO_ADAPT_MIN_MOTION` | 0.1 m | 0.0 - 0.5 | below this the model deviation is not updated |

### Extrinsics and per-dataset

| parameter | env | notes |
|---|---|---|
| `baselink2lidar_pose_str` | `CTLIO_BASELINK2LIDAR` | Oxford Spires needs `0 0 0.124 180 0 0`. Getting this wrong costs almost nothing in APE and everything in RPE, so check RPE when changing it |
| `baselink2imu_pose_str` | `CTLIO_BASELINK2IMU` | |
| `fallback_scan_period` | `CTLIO_FALLBACK_PERIOD` | 0.1 s; used only when the scan carries no usable per-point time field |

## Running on Oxford Spires, and what was verified against ground truth

```bash
SEQ=/mnt/datasets/public/oxford-spires/2024-03-13-observatory-quarter-01
BAG=$SEQ/raw/ros2bag/1710338090_2024-03-13-13-54-51

# The bag carries no /tf, so BOTH fixed-pose flags are required. Without the
# LiDAR one every cloud is dropped before it reaches the module and the run
# ends with an empty trajectory.
export MOLA_USE_FIXED_LIDAR_POSE=true
export LIDAR_POSE_X=0 LIDAR_POSE_Y=0 LIDAR_POSE_Z=0
export LIDAR_POSE_YAW=0 LIDAR_POSE_PITCH=0 LIDAR_POSE_ROLL=0

# T_base_imu, i.e. the IMU in the BODY frame, not in the LiDAR frame. It is
# T_base_lidar * T_lidar_imu, where the other wrappers in the suite quote
# T_lidar_imu = 0.018771 -0.008218 -0.070474 -90.6263 -0.1665 -0.1287.
export MOLA_USE_FIXED_IMU_POSE=true
export IMU_POSE_X=-0.018771 IMU_POSE_Y=0.008218 IMU_POSE_Z=0.053526
export IMU_POSE_YAW=89.3737 IMU_POSE_PITCH=-0.1665 IMU_POSE_ROLL=-0.1287

mola-ct-lio-cli -c pipelines/ctlio-oxford-spires.yaml \
  --input-rosbag2 "$BAG" --lidar-topic /hesai/pandar \
  --imu-topic /alphasense_driver_ros/imu --output-tum-path /tmp/ctlio.tum
```

Two diagnostic dumps exist for checking the inertial path against ground truth
rather than assuming it: `MOLA_CTLIO_DUMP_IMU=<file>` writes the IMU in the
body frame (`t wx wy wz ax ay az`), and `MOLA_CTLIO_DUMP_STATE=<file>` writes
each emitted knot (`t x y z vx vy vz ba bg inliers chi2`).

**The body-frame conversion is verified**, not assumed: with the extrinsic
above, the transformed gyroscope correlates with the ground-truth angular
velocity at 0.99-1.00 on all three axes, with residual means below
0.004 rad/s. Redo that check after touching the extrinsic or the transformer;
a wrong frame does not fail, it just degrades the trajectory.

**The initial bias measurement cannot be trusted on this dataset.** The
calibration gate accepts on the steadiness of the accelerometer *direction*,
which a platform yawing about gravity satisfies perfectly. On
observatory-quarter-01 the rig is turning at 2.7-4.8 deg/s during the window,
and the "bias" comes out at 5.9 deg/s, some 25x the true value. Hence
`max_initial_gyro_bias` / `max_initial_accel_bias`: a measurement beyond a
plausible magnitude is discarded and left to the estimator, which recovers the
true bias within about 2 seconds either way.

## Current accuracy: NOT competitive, and it diverges

Read this before quoting any number from this package.

Full-sequence APE rmse, against the stored corpus in
`/var/www/status/slam-quality/versions/`:

| sequence | this | best stored on it |
|---|---|---|
| oxford obsq-01 | **1.324** | 0.0628 (mola-lo), 0.0641 (fastlio2), 0.0931 (dlio) |
| oxford obsq-02 | **4.701** | - |
| oxford keble-02 | **216.6** | diverges here |
| grand-tour 2024-10-01 | **3.079** | - |
| grand-tour 2024-11-02 | **3865** | diverges here |

An earlier 0.039 m on obsq-01 was measured on a 42 s window and tuned on that
same window. It does not generalize; do not quote it.

The machinery is not the problem. On obsq-01 the per-knot speed matches ground
truth to 1-2% across the whole 290 s with no sustained departure. The failures
are robustness, and both sit at the edges rather than in the middle:

- **keble-02** runs away exponentially from 96% of the way through, the step
  growing about 1.2x per knot. That is an unstable feedback loop.
- **grand-tour 2024-11-02** settles into a sustained 5 m/s from t=118 s, which
  a legged robot cannot do: it lost tracking and kept integrating.
- Every sequence shows a cold start, the first emitted pose being 18x to 51x
  too fast.

There is no divergence guard anywhere in the estimator. That is the first
thing to add.

## The LiDAR/IMU weighting, and why the default is not the datasheet figure

This is the single largest effect measured so far, and it is a balance
problem rather than a bug in either term.

The inertial factors are whitened by a genuine noise covariance. The LiDAR
block is not: GICP's `cov_inv` is a surface-shape matrix built from local
point scatter, and its absolute scale has no particular relation to a
measurement noise. At the accelerometer's nominal `2e-3`, an IMU factor over a
40 ms knot pair carries roughly `1e10` of position information against the
whole LiDAR block's `1e5`-ish, so the trajectory follows the integrated IMU
and the geometry cannot pull it back. What it looks like from outside is not a
failure: inliers and chi2 stay flat while the trajectory quietly under-travels.

Measured on observatory-quarter-01, APE against ground truth:

| `accel_noise_density` | APE rmse | path ratio |
|---|---|---|
| 2e-3 (datasheet) | 0.737 m | 0.970 |
| 1e-2 | **6.285 m** | 2.637 |
| **5e-2 (default)** | **0.039 m** | **1.003** |
| 2e-1 | 0.041 m | - |

For reference on the same window, the same code with `use_imu: false` scores
0.054 m but with a path ratio of **1.78**, i.e. it is accurate on average and
jittery by 2-3 cm per knot. The inertial arm at 5e-2 is better on both counts,
which is what the coupling is supposed to buy.

**Do not read this axis as monotonic.** 1e-2 is not between its neighbors, it
is a blow-up, so a sweep of it needs the intermediate points rather than the
ends.

The proper fix is to make the two blocks commensurate instead of tuning one
against the other. `mp2p_icp` already has the machinery: `Solver_GaussNewton`
carries a Birge-ratio auto-balance of the cov2cov block against its prior, for
exactly this reason. Adopting that here would replace this parameter. Until
then, treat 5e-2 as an effective weight that absorbs scale factor,
misalignment, carrier vibration and the units mismatch, not as a claim about
the sensor.

## What the system test establishes

`test_odometry_engine` simulates a platform starting from rest and
accelerating, and measures the emitted trajectory against the truth. Two
results worth keeping in mind when reading any later number:

- In a scene with structure in every direction, LiDAR-only and LiDAR-inertial
  both track to under 5 cm over ~1.8 m.
- In a **corridor**, where every surface runs along the direction of travel,
  axial motion is not observable from the geometry at all. LiDAR-only drifts
  (~0.2 m); with the IMU the same run finishes at **0.6 mm**. That is the test
  that says the inertial coupling carries information rather than merely being
  wired up, so treat it as the regression guard on the IMU path.

A cold start is a real limitation, not a bug: an odometry cannot know it was
already moving at the first scan, so a sequence that begins at speed loses
whatever it travels before the window first converges. Real sequences start
near rest.

## Where the time goes, measured

Set `CTLIO_PROFILE=true` for a per-stage table at the end of a run (MRPT
`CTimeLogger`). On Oxford observatory-quarter-01, 255 scans:

| stage | before | after | note |
|---|---|---|---|
| `mapInsert` | 38.8 s | **4.0 s** | was pruning the map on every 40 ms segment |
| `optimizeWindow` | 39.6 s | 31.1 s | of which `.match` 21.2 s |
| ↳ `match.nnSearchCov2Cov` | 19.1 s | 18.4 s | the GICP search, inside the map class |
| ↳ assembly + solve | ~18.2 s | ~9.9 s | TBB deterministic reduce |
| `marginalize` | 0.05 s | 0.07 s | negligible, despite being the scary part |

Two lessons worth keeping. Nearly half the runtime was in **map pruning**, not
in anything algorithmic: `keepOnlyPointsNear()` rebuilds the k-d tree and
recomputes every covariance, and it was being called on every segment while
`map_radius` was larger than the whole trajectory, so it evicted nothing.
Hence `map_prune_period`. And the remaining dominant cost is
`nnSearchCov2Cov`, which lives in `mola_metric_maps`, not here, so profile
before optimizing anything local.

## Parallelism and determinism

The per-point assembly uses `tbb::parallel_deterministic_reduce` with a
**fixed** grain size (512). That is the only reason parallelizing it is
acceptable: it fixes the split points and the reduction tree from the range
and the grain alone, so the summation order does not depend on the thread
count or on task stealing. An automatic grain would give that up.

Verified end to end, not assumed: the same sequence run on 88, 4 and 1 core
produces **byte-identical** trajectories. `test_ct_normal_equations` guards the
same property at the unit level via `tbb::global_control`. Any new parallel
code here has to keep it.

Without TBB the assembly falls back to a serial loop and the build still works.

## Matching cost, measured

`CtMapMatcher.TheEstimatorRecoversAPerturbedSegmentThroughRealMatching` runs
20 iterations over one ~500-point segment in 205 ms, i.e. **~10 ms per
iteration**, nearly all of it in building the local map's k-d tree and
covariances. A three-segment window re-matching every iteration would
therefore cost about 600 ms per 120 ms of data, which is why `rematch_every`
exists. Re-measure this before tuning it on real data; the synthetic segment
is smaller than an Oxford one.

## Two facts the matching layer depends on

Both checked in `mola::IncrementalPointCloud`, both worth re-checking if that
map or `Matcher_Cov2Cov` changes underneath us.

- `point_with_cov_pair_t::local_idx` is the **storage slot** of the local map.
  A freshly built map has no tombstones, so the slot equals the insertion
  order, which is what lets a pairing be mapped back to the source point's
  `alpha`. Reusing a local map across segments instead of rebuilding it would
  break that identity.
- The pairings come back **sorted by `local_idx`**, a total and canonical
  order that does not depend on the tree shape or the thread count. The
  assembly's summation order therefore inherits determinism for free with
  this matcher. A different matcher has to be checked for the same property
  before it can be used in a deterministic run.

`point_plane_pair_t` carries **no index**, so `Matcher_Points_KnnPlane`
cannot currently be mapped back to `alpha`. Using it needs a small upstream
`mp2p_icp` change first.

## Dataset gotchas carried over from the wrapper work

- GrandTour rigs use `base`, not `base_link`: export `MOLA_TF_BASE_LINK=base`.
- **GrandTour Hesai clouds are already motion compensated by the provider.**
  The only topic published is `/boxi/hesai/points_undistorted`, and it still
  carries a per-point time field, so the times look perfectly usable while the
  geometry has already been corrected. Deskewing again would apply the
  correction twice. Hence `clouds_already_deskewed: true` in that pipeline,
  which reads each scan as a single-instant observation, with the knot spacing
  set to the scan period so one scan constrains one knot. It costs this method
  its continuous-time advantage on that dataset; that is a property of the
  data, not a choice. A raw distorted bag would restore it, and none is
  published.
- The LiDAR extrinsic comes from the observation's own sensor pose, which the
  dataset source fills from `/tf` or from its fixed-pose configuration;
  `baselink2lidar_pose_str` is an *extra* transform composed on top of it. A
  bag with `/tf` (GrandTour) needs the parameter at identity; a bag without one
  (Oxford Spires) supplies identity and the parameter carries the whole
  extrinsic.
- KITTI carries no per-point timestamps at all, so every KITTI number is a
  no-deskew number and the continuous-time part degenerates to the azimuth
  fallback there.
- A per-point time field can be present and useless: MRPT holds those as
  `float`, and an absolute Unix time near 1.7e9 quantizes a whole sweep onto
  one value. Check the span covers several representable steps of the field.
