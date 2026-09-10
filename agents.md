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
| `accel_noise_density` | `CTLIO_ACCEL_SIGMA` | 2.0e-3 | 1e-4 - 1e-2 | |
| `bias_sigma_acc` | `CTLIO_BIAS_SIGMA_ACC` | 1e-3 | 1e-5 - 1e-2 | bias random walk density |
| `bias_sigma_gyro` | `CTLIO_BIAS_SIGMA_GYRO` | 1e-4 | 1e-6 - 1e-3 | |
| `imu_init_seconds` | `CTLIO_IMU_INIT_SEC` | 1.0 | 0.5 - 3.0 | static window for initial attitude and bias |

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
- KITTI carries no per-point timestamps at all, so every KITTI number is a
  no-deskew number and the continuous-time part degenerates to the azimuth
  fallback there.
- A per-point time field can be present and useless: MRPT holds those as
  `float`, and an absolute Unix time near 1.7e9 quantizes a whole sweep onto
  one value. Check the span covers several representable steps of the field.
