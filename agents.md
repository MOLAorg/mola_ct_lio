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
| `pipelines/` | per-dataset configuration: `common/ctlio-base.yaml` holds every knob, each file next to it `$import`s that base and `$define`s only what its dataset changes. `ctlio-generic.yaml` overrides nothing and is what an unknown rig runs |
| `mola-cli-launchs/` | MOLA launcher configs: `ct_lio_from_rosbag{1,2}.yaml` are the online, GUI counterparts of the offline CLI |
| `scripts/` | dataset wrappers (`mola-ct-lio-{cli,gui}-<dataset>`) over the profiles in `scripts/lib/`, plus the scoring and bench tools |

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
follow the `${CTLIO_NAME|default}` pattern, and all of them live once in
`pipelines/common/ctlio-base.yaml`; a dataset file changes one by `$define`-ing
the same name, which the environment still overrides. Add a parameter to the
base file and to this table together.

### Trajectory and window (`WindowOptimizer::Params`, estimator)

| parameter | env | default | sweep range | notes |
|---|---|---|---|---|
| `segment_interval` | `CTLIO_SEG_INTERVAL` | 0.04 s | 0.02 - 0.10 | knot spacing; upstream Traj-LO uses 0.04 |
| `knot_count` | `CTLIO_KNOT_COUNT` | 4 | 3 - 8 | window size; upstream uses 3 control poses |
| `init_interval` | `CTLIO_INIT_INTERVAL` | 0.3 s | 0.1 - 1.0 | data gathered before the first solve |
| `max_iterations` | `CTLIO_MAX_ITERS` | 25 | 5 - 40 | |
| `convergence_threshold` | `CTLIO_CONVERGE_TH` | 1e-3 m | 1e-4 - 1e-2 | on the largest knot translation step |
| `lambda` | `CTLIO_LAMBDA` | 0.0 | 0 - 1e-3 | Levenberg damping as a fraction of the diagonal; 0 is plain Gauss-Newton |
| `max_step_translation` | `CTLIO_MAX_STEP` | 1.0 m | 0.2 - 5.0 | trust region: a longer step is scaled down as a whole. 0 disables it |
| `max_step_velocity` | `CTLIO_MAX_STEP_V` | 2.0 m/s | 0.5 - 10 | the same bound on the velocity block, which the translation one does not cover |
| `relinearize_each_slide` | `CTLIO_RELIN` | false | false / true | whether a knot's Jacobian point follows the estimate or is held from its first marginalization |
| `velocity_prior_sigma` | `CTLIO_VEL_PRIOR` | 5.0 m/s | 2 - 20 | absolute bound on the velocity state; the step bound is not one |
| `max_imu_wait_segments` | `CTLIO_IMU_WAIT` | 3.0 | 1 - 10 | how long a ready segment waits for the inertial stream, in segments |
| `min_imu_coverage` | `CTLIO_IMU_COVERAGE` | 0.98 | 0.9 - 1.0 | fraction of a segment the preintegration must span to be used |
| `bias_prior_sigma_acc` | `CTLIO_BIAS_PRIOR_ACC` | 0.3 m/s^2 | 0.1 - 1.0 | absolute bound on the accel bias; the random walk alone leaves it unbounded. 0 disables |
| `imu_time_offset` | `CTLIO_IMU_DT` | 0.0 s | -0.02 - 0.02 | added to every inertial sample's stamp; temporal calibration |
| `odometry_sigma_lin` | `CTLIO_ODO_SIGMA_LIN` | 0 (off) | 0.005 - 0.5 | external odometry's relative motion per segment. 0 disables |
| `odometry_sigma_ang` | `CTLIO_ODO_SIGMA_ANG` | 0 (off) | 0.002 - 0.2 | same, rotation |
| `bias_prior_sigma_gyro` | `CTLIO_BIAS_PRIOR_GYRO` | 0.02 rad/s | 0.005 - 0.05 | same for the gyro bias |
| `starvation_ratio` | `CTLIO_STARVATION` | 0 (off) | 0 - 0.5 | a segment holding this fraction of the recent average is held out of the map. 0 disables |
| `lidar_balance_min_dof` | `CTLIO_BALANCE_MIN_DOF` | 200 | 50 - 1000 | degrees of freedom the LiDAR block needs before its reduced chi-square is acted on |
| `segment_phase_offset` | `CTLIO_SEG_PHASE` | 0.0 | 0.0 - 0.5 | where in a segment the first scan lands. Only worth moving when a provider gives one instant per scan |
| `lidar_balance` | `CTLIO_LIDAR_BALANCE` | TwoSided | None, DownOnly, TwoSided | reconciles the LiDAR block's weight with its own residuals, see below |
| `lidar_balance_max_scale` | `CTLIO_LIDAR_BALANCE_MAX` | 1000 | 100 - 1e4 | how far the balance may rescale the block in either direction |
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
| `source_voxel_stride` | `CTLIO_DS_STRIDE` | 1 | 1 - 3 | keep one occupied source voxel in this many; thins without coarsening |
| `min_segment_points` | `CTLIO_MIN_SEG_PTS` | 0 Oxford / 5000 grand-tour / 2500 KITTI | 0 - 5000 | a short segment is re-decimated on a finer cell until it clears this. 0 disables |
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
| `threshold` | `CTLIO_MATCH_TH` | 0.80 m | 0.4 - 1.5 | also the near value when `thresholdFar` is set |
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

## Running on GrandTour

Both arms are one command, and both read the same profile, so they cannot
disagree about a bag name, a topic or a frame:

```bash
mola-ct-lio-cli-grandtour /data/grand-tour/2024-10-01-11-29-55/   # batch, writes a TUM file
mola-ct-lio-gui-grandtour /data/grand-tour/2024-10-01-11-29-55/   # replay with the 3D GUI
```

Either accepts a mission directory or any one of its bags, and both honor
every `CTLIO_*` knob from the environment, so a sweep is a loop around one of
them. The dataset choices live in `scripts/lib/profiles/grandtour.sh`
(`MOLA_GRANDTOUR_LIDAR`, `MOLA_GRANDTOUR_IMU`, the camera preview); run either
with no arguments for the full list. Adding a dataset means one more profile
file and one more name in the CMake wrapper list.

The TF bag is required rather than optional, which is where this differs from
the LiDAR-only wrappers: without `/tf` the IMU is placed at the vehicle origin,
and the wrong lever arm produces a plausible trajectory rather than a failure.

## Running on your own bag

The `rosbag2` wrappers are not tied to a dataset: topics, /tf names and every
pipeline knob come from the environment, and the pipeline they run
(`ctlio-generic.yaml`) overrides nothing.

```bash
MOLA_LIDAR_TOPIC=/velodyne_points MOLA_IMU_TOPIC=/imu/data \
MOLA_TF_BASE_LINK=base_footprint \
  mola-ct-lio-cli-rosbag2 ~/bags/my-recording/      # or -gui- to watch it
```

`MOLA_TF_TOPIC` / `MOLA_TF_STATIC_TOPIC` handle a bag recorded under a
namespace, where the transforms arrive as `/robot1/tf` and a reader looking at
`/tf` finds no extrinsics rather than failing. An empty `MOLA_IMU_TOPIC`
selects the LiDAR-only arm. A bag with no `/tf` at all needs the fixed poses
instead (`MOLA_USE_FIXED_IMU_POSE=true` plus `IMU_POSE_*`); run either wrapper
with no arguments for the whole list.

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

Residual-driven balancing changed the picture completely. With
`lidar_balance: TwoSided` and otherwise identical settings on every dataset:

| sequence | before | now | best on record |
|---|---|---|---|
| obsq-01 | 1.456 | **0.0748** | 0.0628 (lio cfg-03) |
| obsq-02 | 1.546 | **0.1595** | 0.0543 (lio cfg-01) |
| keble-02 | 1.153 | **0.0433** | **this**, next 0.0463 |
| grand-tour 10-01 | 2.920 | **0.0990** | |
| grand-tour 11-02 | 544.8 | **1.198** | |

Nothing diverges any more, and keble-02 is the best number on record for that
sequence. obsq-02 is the laggard and grand-tour 11-02 is still an order of
magnitude off the Oxford sequences.

What got there, in order of effect: the residual-driven LiDAR balance, then
the absolute bias prior, then a consistent marginalization recursion.

- **The balance.** See the weighting section below. Largest single effect by
  far, and it removes a hand-tuned constant rather than adding one.
- **An absolute prior on the IMU bias.** The random-walk factor only speaks
  about the *difference* between consecutive knots' biases, so the biases
  themselves were unbounded. On a diverging sequence the accelerometer bias
  goes from 0.14 to 25 m/s^2 in one second, two and a half times gravity, and
  stays there; velocity, inliers and residual all follow it rather than lead
  it. A control run with the prior disabled reproduces the old number exactly.
- **A consistent marginalization recursion.** The prior used to be built from
  the system as it stood one step before the states it was declared to
  describe, and the Jacobian point was recaptured on every slide so each prior
  inherited Jacobians taken elsewhere. Holding the Jacobian point is worth 13x
  on keble-02 on its own.

`max_step_translation` is insurance, not a fix: it never fires on any sequence
measured so far. The runaways are slow ramps, not jumps.

The cap is not what these numbers rest on. Raising
`lidar_balance_max_scale` from 100 to 1000, where it binds 28% of windows
rather than 91%, moves obsq-01 by 0.0002, improves keble-02, grand-tour 10-01
and 11-02, and costs obsq-02 5%. 1000 is the default on that balance.

The balance also did what a hand-tuned constant never could: it made the
inertial noise density stop mattering. Over a 20x range of
`accel_noise_density`, with everything else fixed:

| accel_noise_density | obsq-02 before | obsq-02 after | keble-02 after |
|---|---|---|---|
| 1e-2 | 66.1 | 0.173 | 0.0426 |
| 5e-2 | 1.546 | 0.1595 | 0.0433 |
| 2e-1 | 853 | 0.1315 | 0.0455 |

Four orders of magnitude of swing became thirty percent. That, rather than any
single APE figure, is the robustness result.

### The bias prior wants to be tight, not loose

obsq-02 was the laggard, and its accelerometer bias was the one thing that set
it apart: a median norm of 0.39 m/s^2 against 0.08 to 0.13 elsewhere. The
per-axis medians are small, so it is not an offset; the norm is made of
excursions reaching 1.09 m/s^2. The state was absorbing registration error,
not sensor bias, and the remedy is the opposite of the obvious one:

| `bias_prior_sigma_acc` | obsq-02 |
|---|---|
| 2.0 | 0.391 |
| 1.0 | 0.386 |
| 0.3 | 0.1595 |
| 0.05 | 0.1467 |

Loosening the bound nearly triples the error. Tightening the bias random walk
alongside it adds nothing (0.148 against 0.147), so it is the absolute bound
doing the work.

## What grand-tour actually fails on: transient point starvation

All ten missions with ground truth, full sequences, current defaults, scored
with the body-frame correction below:

| mission | APE | median segpts | p10 segpts |
|---|---|---|---|
| 2024-11-03-07-57-34 | 0.022 | 6189 | 3957 |
| 2024-11-14-13-45-37 | 0.027 | 7516 | 5605 |
| 2024-11-03-13-51-43 | 0.052 | 5234 | 3310 |
| 2024-10-01-11-29-55 | 0.052 | 7400 | 4154 |
| 2024-12-09-09-41-46 | 0.294 | 5372 | 3719 |
| 2024-12-09-09-34-43 | 0.295 | 5352 | 3953 |
| 2024-11-02-17-18-32 | 1.323 | 896 | 580 |
| 2024-12-09-11-28-28 | 1.614 | 1700 | 381 |
| 2024-11-18-13-22-14 | 4.915 | 458 | 215 |
| 2024-11-18-13-48-19 | 10.124 | 3555 | 266 |

The tenth percentile of a segment's point count separates the failures
exactly: every mission above 0.5 m has a p10 under 600, every one below has a
p10 over 3300. The *median* does not, and that is the whole point:
2024-11-18-13-48-19 has a perfectly healthy median of 3555 and is the worst
result of the ten. What breaks these runs is not a sparse sensor but
occasional segments arriving nearly empty, and nothing in the estimator
currently treats such a segment differently from a full one. Four of the ten
are at or below 0.052 m, so the machinery is right when it is fed.

## Decimation has to follow density, and a fixed voxel cannot

Finer decimation (0.15 m source, 0.2 m map) against the 0.4 m default:

| sequence | median segpts | 0.4 m | 0.15 m |
|---|---|---|---|
| obsq-01 | 3116 | 0.0753 | 0.1437 |
| keble-02 | 2690 | 0.0431 | 0.0762 |
| obsq-02 | 3193 | 0.1302 | 0.1507 |
| grand-tour 2024-10-01 | 7400 | 0.0521 | 0.0696 |
| grand-tour 2024-11-03-07-57-34 | 6189 | 0.0222 | 0.0222 |
| grand-tour 2024-12-09-09-34-43 | 5352 | 0.295 | 0.294 |
| grand-tour 2024-11-02 | 896 | 1.323 | 0.892 |
| grand-tour 2024-12-09-11-28-28 | 1700 | 1.614 | 0.300 |
| grand-tour 2024-11-18-13-22-14 | 458 | 4.915 | 0.954 |

The split is by density and nothing else: every sequence above about 2500
points per segment is unchanged or worse, every one below is better, by five
times on the two sparsest. So neither value is a defensible default, and the
knob is the wrong shape. What the data asks for is a target *count* rather
than a fixed cell: coarse where returns are plentiful, fine where they are
scarce. `source_voxel_stride` is already the mechanism for the thinning half.

A longer segment is not the answer for the sparse missions either: 0.20 s
takes 2024-11-02 from 1.32 to 0.78 but 2024-11-18-13-22-14 from 4.92 to 5.99,
and 0.30 s is far worse on both.

So the knob became a floor rather than a target. A segment that comes out
short is re-decimated on a cell halved up to three times, stopping as soon as
it clears the floor or the finer cell stops adding points. A segment already
above the floor never enters that path at all, which is what keeps the
sequences this corpus already handles well bit-for-bit unchanged.

## Withholding a starved segment from the map makes things worse

Worth recording as a dead end, since the reasoning is appealing. A segment
registered on very little geometry has a poorly determined pose, so its points
enter the map at that pose in front of every scan that follows; holding it out
should stop a brief loss of returns becoming a lasting one. Measured, it does
the opposite:

| mission | class | before | withheld |
|---|---|---|---|
| 2024-11-18-13-48-19 | episodic | 10.124 | 10.802 |
| 2024-12-09-11-28-28 | episodic | 1.614 | 1.871 |
| 2024-11-03-07-57-34 | healthy | 0.022 | 0.149 |
| 2024-10-01-11-29-55 | healthy | 0.052 | 0.052 |

It is worse on both missions it was built for and costs a healthy one a factor
of seven. Sparse points are worth more than the pose error they carry, and a
map that is not fed is a worse problem than a map fed slightly wrong. Left in
at `starvation_ratio: 0`.

The grand-tour failures also split into two classes that want opposite things,
which is why one guard was never going to serve both. Comparing p10 against
the median: 2024-11-02 (896, 0.65) and 2024-11-18-13-22-14 (458, 0.47) are
*uniformly* sparse, while 2024-11-18-13-48-19 (3555, 0.07) and 2024-12-09-11-28-28
(1700, 0.22) have healthy medians with severe dips. A relative test cannot
fire on the first pair by construction, and indeed 2024-11-02 came back
byte-identical. Sustained sparsity wants more geometry per segment instead.

The one piece worth keeping from that round is `lidar_balance_min_dof`: the
balance used to act on a reduced chi-square computed from as few as three
correspondences, whose own relative error goes as `sqrt(2/dof)`, and was free
to amplify a near rank-deficient block by up to the cap. It is inert on every
sequence measured so far, where the block carries tens of thousands of degrees
of freedom, so it is insurance rather than a fix.

## How a deskewed window actually dies, and what the trust region missed

Worth recording in full, because the first two explanations were wrong.

Per-point deskew plus the inertial term diverges on grand-tour. It is not the
raw bags: the provider's own clouds do the same through that path. It is not
the deskew: the same points with the inertial term off score 0.162 m. Reading
the run knot by knot shows the actual sequence:

| knot | segpts | inliers | speed | bias |
|---|---|---|---|---|
| 1-6 | ~5000 | ~20000 | 0.13-0.17 | 0.2-0.37 |
| 7 | 5428 | 18139 | **37.90** | 0.160 |
| 13 | 5362 | 11853 | 35.48 | 3.23 |
| 40 | 3214 | **0** | 93158 | -- |

The velocity state jumps by three orders of magnitude in one window *while the
geometry is still healthy*, with eighteen thousand inliers. Everything after
that, the bias climbing past its own prior and the correspondences vanishing,
is consequence rather than cause.

The trust region should have caught it and could not: it only measured the
step's position block. A step whose translation is unremarkable and whose
velocity block is enormous passed it untouched, and on that run it fired on
2.9% of windows. Velocity is precisely the state that an inconsistency
between the geometry and the inertial term collects in, so it needs its own
bound, which `max_step_velocity` now gives it.

## ARC-7's divergence is the inertial stream ending before the LiDAR's

Comparing the estimate against the legged estimator over one-second windows,
which removes both sides' long-run drift and asks only whether they agree on
how far the robot went:

| | |
|---|---|
| median disagreement over 334 s | **0.009 m** |
| windows disagreeing by more than five times that | 12, all of them after t=321 s |
| what the legged estimator reports there | 0.000 m, the robot is standing still |

So the run is essentially perfect for 96% of its length and then leaves in the
last thirteen seconds. The bags say why: the Hesai's last message is at
1731947868.2 and the ADIS's is at 1731947855.8, twelve and a half seconds
earlier, and t=321 s is exactly that instant. The final stretch runs with no
inertial factor at all, which on these missions diverges.

The estimator had no answer for that. The twist-continuity term that stands in
for the IMU was applied only when the whole run was non-inertial, so a segment
that lost its inertial factor mid-run received nothing and the window was free
to move wherever the geometry did not pin it. That mattered more once
`min_imu_coverage` began refusing factors. It now applies whenever any segment
in the window lacks one.

Worth carrying to any dataset: check that the inertial stream outlasts the
LiDAR before trusting the tail of a run, and compare against a
non-geometric reference over short baselines to find *when* an estimate left,
which an absolute-error metric will not tell you.

## Part of what is being measured on grand-tour is the reference itself

Its tier-1 reference, the RTK-INS at `cpt7_imu`, carries **0.132 m mean ATE by
upstream's own validation**. It is decimeter-class, not centimeter. The tier-2
reference, the total-station prism, is far better but position-only.

That splits this dataset's missions into two groups whose numbers cannot be
read the same way:

| reference | missions | ours |
|---|---|---|
| prism, tier 2 | con-1, con-2, con-3 | 0.023, 0.026, 0.017 |
| RTK-INS, tier 1 | everything else | 0.021 to 0.24 |

On spx-2 we score 0.1613 against a reference whose own error is 0.132. If the
two are independent, our own contribution is about
`sqrt(0.161^2 - 0.132^2) = 0.09` m, and most of what the metric reports is not
ours.

This is the explanation for an otherwise baffling sweep. Eight parameters that
set local registration precision -- neighbours for the covariance, rematch
period, window length, kernel scale, correspondence distance, plane deviation
-- move spx-2 between 0.1608 and 0.1614, a spread of four tenths of a percent.
Nothing moves it because the number is not mostly about us.

Two consequences. Tuning against tier-1 missions past roughly 0.13 m is
chasing the reference's noise, and a local dev mean over those missions will
not predict a benchmark scored against better hidden references. Prefer the
prism missions, and prefer relative metrics, for anything meant to detect a
real improvement.

## The residual error is local, not a warp, which inverts the usual advice

Relative translation error over a 10 m baseline, against absolute error, on
the shipped defaults:

| mission | ATE | RTE(10 m) | RTE/ATE |
|---|---|---|---|
| spx-2 | 0.1613 | 0.1309 | 0.81 |
| arc-2 | 0.2402 | 0.3021 | 1.26 |
| eig-1 | 0.0526 | 0.1289 | 2.45 |
| snow-2 | 0.0210 | 0.0873 | 4.16 |

The reference method's own submission sits at 0.29 and the rest of that
leaderboard at 0.44 to 0.68, and the conclusion drawn there was that local
registration is not the lever, the error being a smooth place-dependent warp.
This estimator is the opposite: the local error is *larger* than the global
one, so local registration is exactly the lever here and that advice does not
transfer.

Two further measurements say the same thing from the other side. Compared
against the legged estimator over one-second windows, arc-2 and spx-2 have no
disagreement above five times the median anywhere, 0 windows of 436 and 394
respectively, at a median of 8 mm and 6 mm. So there are no episodes left,
only accumulated drift, and the drift is coming from trajectory that is locally
noisy rather than globally bent.

A likely cause is specific to this dataset. With one instant per scan a
segment carries a single alpha, so the LiDAR constrains the pose at that alpha
and says nothing about the motion between the knots, which is free to wobble.
That looked like an argument for the deskewed path, now that the reason it
used to diverge is understood and fixed. Measured, it is not:

| spx-2 | ATE | RTE(1 m) | RTE(10 m) |
|---|---|---|---|
| single instant | 0.1613 | 0.0223 | 0.1309 |
| per-point deskew | 0.1613 | 0.0229 | 0.1287 |

Deskewing gives every segment a full alpha spread where it had one value, and
changes nothing on either metric. So a single alpha is not what makes this
trajectory locally noisy, and the reasoning above, however plausible, was
wrong.

What the deskewed run does show is that the path itself is now sound: it went
from 2.6e6 m to parity with the single-instant result. It simply buys nothing,
while costing 24 GB of raw bags and a good deal more compute, so the
single-instant path stays.

## The deskewed inertial path: a preintegration that does not span its segment

This is the defect behind every deskew symptom recorded below, and it is a
plumbing invariant rather than anything in the mathematics.

A point that carries its own capture time sits up to a full sweep after the
message that delivered it. Bags replay in record order, so when a scan
arrives the inertial deque holds samples only up to about that message's own
stamp. `closeReadySegments()` closes on the latest *point* time, so it closes
segments up to 99.6 ms past the last inertial sample available;
`preintegrate()` clamps to what it has and returns whatever it managed to
cover; and `hasImu = imu.dt > 0` accepted any of it.

The factor then describes a different interval from the one its two knots are
apart. Two consequences, both observed:

- Every factor short by roughly half an inertial period leaves gravity
  under-compensated by 0.025 m/s per segment, a steady push only the
  accelerometer bias can absorb. That is the bias climbing against its own
  prior.
- A scan gap makes one call close two segments, and the second gets a few
  milliseconds. The position residual then reads the segment's whole
  displacement as if it happened in that time, so velocity inflates by the
  ratio. Preintegration covariance shrinks as `dt^3`, so such a factor carries
  enormous information: it is not a weak measurement, it is a maximally
  confident wrong one.

Measured over whole missions at a 0.1 s knot spacing, about 4% of factors were
being dropped outright and a few tenths of a percent kept with under 20 ms of
coverage, the worst inflating by 1700x to 1900x. With one instant per scan the
coverage is 1.000 everywhere, which is the entire difference between the two
paths.

It also explains why sweeping `imu_time_offset` showed no minimum. The offset
shifts the inertial frontier, so it shifts coverage rather than any physical
alignment: the two settings that diverge are exactly the two whose worst
inflation runs into the thousands, and the best of the sweep is the only one
with no truncated factor at all.

The fix is to hold a ready segment until the inertial stream reaches its end,
bounded so a stalled stream cannot stop the trajectory, and to refuse a
preintegration that spans less than `min_imu_coverage` of its segment.
`imuCoverage` is now in the state dump, since nothing reported it before,
which is why this survived so long.

## A five-millisecond clock offset, and why it only shows up now

Verified against this dataset's own reference, with the gyro rotated from
`adis16475_imu` into `box_base` by the bag's own `/tf_static`, the ADIS agrees
with ground-truth angular velocity as follows:

| shift | x | y | z |
|---|---|---|---|
| none | +0.857 | +0.947 | +0.988 |
| +5 ms | +0.952 | +0.987 | +0.992 |

Five milliseconds is half a segment at the 0.1 s knot spacing and more than a
whole segment at 0.04 s. A continuous-time estimator is unusually exposed to
this. With one instant per scan the LiDAR says nothing about motion *inside* a
segment, so a clock offset merely biases the answer; once the points carry
their own times both sensors constrain the same motion and an offset is a
direct contradiction between them.

That is what the deskew experiments show. Turning the per-point deskew on
diverges with the inertial term (2.6e6 m on the provider's own clouds, so it
is nothing to do with the raw bags) and behaves perfectly without it *on one
mission*: raw scans, real deskew, LiDAR only scores 0.162 m on spx-2. That
single result does not generalize, and reading it as "the inertial term is
harmful" was wrong. Measured on five missions, LiDAR-only diverges on three of
them and is two to three times worse on the rest:

| mission | LiDAR only | with IMU |
|---|---|---|
| arc-2 | 2811 | 0.858 |
| arc-3 | 9117 | 2.459 |
| con-3 | 5038 | 0.0167 |
| snow-2 | 0.0597 | 0.0208 |
| eth-1 | 0.1348 | 0.0521 |
| spx-2 | 0.162 | 0.161 |

The inertial term is load-bearing everywhere except the one sequence that
happened to be measured first. So the deskewed path has to be fixed rather
than traded away. `imu_time_offset` corrects it, and the
correction is what decides whether the deskewed path runs at all:

| | dt = 0 | dt = +5 ms |
|---|---|---|
| spx-2 | 64553 | **2.263** |
| arc-2 | 28699 | **1.625** |

Four orders of magnitude from five milliseconds. Both halves were needed to
get there: bounding the velocity step alone cut the blow-up from 1.0e7 to
6.5e4 and still left it divergent, and the offset alone was measured against
the reference but never acted on. The offset is the seed, the unbounded
velocity step was the amplifier.

Sweeping the offset properly, however, says this is not a calibration
problem after all:

| offset | spx-2 |
|---|---|
| -10 ms | 1.362 |
| 0 | 64553 |
| +3 ms | 84513 |
| +5 ms | 2.263 |
| +7 ms | 2.369 |
| +10 ms | 2.011 |
| +15 ms | 2.961 |
| +20 ms | 2.338 |
| *single instant, no deskew* | ***0.161*** |

There is no optimum. Two settings three milliseconds apart give 84513 m and
2.26 m, and the best deskewed result of the whole sweep is eight times worse
than the single-instant path already in use. A well calibrated clock would
show a smooth minimum; this shows a configuration that happens to survive at
some offsets and not others.

So the raw bags buy nothing today, and the conclusion the four-orders-of-
magnitude result invited -- that the clock was the fault and correcting it
fixes the deskewed path -- is not supported. The offset is real and measured,
the velocity bound is right on its own terms, and the deskewed inertial path
remains marginally stable for a reason not yet found. Grand-tour continues to
run single-instant.

## External odometry: built, measured, and switched off for now

The legged platform publishes its own kinematic-inertial estimate at 20 Hz,
and the relative-motion factor that consumes it works: 5% on arc-2, 8% on
arc-3, neutral on spx-2 and snow-2. Tighter sigmas are better here, down to
0.005 m / 0.002 rad, which is the opposite of what the reference method found
with the same source.

It is off by default anyway. The LiDAR-inertial core still has failures nobody
has explained -- per-point deskew destabilises the inertial term for reasons
that are not the clock, not the data and not the deskew itself -- and fusing a
second pose source on top of that only makes those harder to read. It goes
back on once the core is understood alone.

## Scoring grand-tour needs the dataset's own body-frame correction

Its ground truth is anchored to a physical sensor mount, not to `base`, which
is what the odometry reports. The mount is not the same for every mission, and assuming it is costs more
than skipping the correction altogether. Most are anchored to the RTK-INS,
`cpt7_imu`, `0.0764 -0.0361 0.2803 0 -0 179.9954` from `base`, a 0.29 m lever
arm and a 180 degree roll. The three construction-site missions are anchored
to the total-station prism instead, `0.3852 0.0022 0.5152 0.5425 0.1622
179.3402`, a 0.64 m arm, and their truth is position-only. Scoring those three
with the RTK offset reads 0.295 m where the right one reads 0.023 m: a
twelvefold error, on a robot that pitches on every step.

The correction is `T_world_sensor(t) = T_world_base(t) . T_base_sensor`, the
arm rotated into the world frame by the estimate's own orientation at each
timestamp. A constant shift will not do, since alignment absorbs that.

It is not a formality: on 2024-10-01 it takes the result from 0.0990 m to
0.0615 m, so scoring without it understates the method by nearly 40%. Use
`score_gt.py`, not the plain scorer, for anything from this dataset.

## The raw grand-tour LiDAR bags exist, and we were not using them

Each mission publishes about thirty per-topic bags, and the subset fetched for
LIO work took `<mission>_hesai_undist.bag`. Upstream also publishes
`<mission>_hesai.bag` (3.06 GB against the undistorted 2.49 GB) and
`<mission>_hesai_packets.bag`, neither of which was downloaded. Listing a
mission's full set: `COLUMNS=250 klein list files -p GrandTourDataset -m
release_<mission>`.

Downloading them needs kleinkram 0.60.0 or newer. An older client reports
`AccessDenied` on every file, which reads like an expired token and is not:
the server's actual reply is `Invalid UUID, 400`, an API mismatch the client
mislabels. `~/kleinkram-venv/bin/pip install -U kleinkram` is the fix, and
files download by id rather than by name.

The raw stream is `/boxi/hesai/points`, with fields `x y z intensity ring
timestamp`, the timestamp a float64 spanning 0.0996 s across a scan: real
per-point timing over a full sweep, about 37000 points per scan. Our reader
normalizes it and reports `per-point relative`, so a segment can be a fraction
of a sweep again.

This matters more than a convenience. Everything below about segment
degeneracy follows from the provider having collapsed each scan to one
instant; raw scans carry their own per-point timing and uncorrected geometry,
which is precisely what a continuous-time method is built to consume.

Worth knowing before reaching for a shortcut: the *undistorted* clouds do
still carry usable per-point timestamps, which our reader reports as
`per-point relative` when `clouds_already_deskewed` is off. They cannot be
used as they stand. Those coordinates have already been moved to a single
reference instant, so evaluating each point at `poseAt(alpha)` applies the
motion correction a second time, of order 0.1 m over a segment against 0.03 m
residuals. The times are real; the geometry they belong to is gone.

## Pre-deskewed clouds make the continuous-time model degenerate

With `clouds_already_deskewed`, every point of a scan carries one timestamp,
because the geometry does belong to one instant. The LiDAR information of a
point at `alpha` splits between the segment's two knots as `(1 - alpha)` and
`alpha`, so a scan sitting on a segment boundary gives the end knot nothing at
all, and a segment holding a single scan has no second alpha to fit an
interpolation to. Scan jitter across a boundary also leaves some segments
empty outright: on grand-tour the median segment holds 866 points against
Oxford's 12000, and some hold none.

Only the undistorted topic exists in these bags, so the sensor's own per-point
timing is gone and cannot be recovered. Two ways out, neither free:

- **A longer segment**, spanning several scans. Sharply non-monotonic, and it
  has to be an integer multiple of the scan period: on grand-tour 2024-11-02,
  0.10 s gives 545 m, 0.15 s (alternating one and two scans) gives 2024 m,
  0.20 s gives 2.44 m, 0.30 s gives 13.5 m and 0.50 s gives 23000 m, the last
  because constant twist stops holding over that long an interval. But 0.20 s
  takes the *other* mission from 2.92 m to 15.6 m, so it is not a safe default.
- **`segment_phase_offset`**, centering an undivided scan between its two
  knots without lengthening the segment. Costs nothing in principle where the
  points already span the segment, but it measurably degrades the synthetic
  IMU-rescue case, so it defaults to off and is opt-in per dataset.

The first segment is the one shortened by the phase offset. Shifting the whole
grid earlier instead would put the anchor knot before any data arrived, which
leaves the origin pose describing an instant nothing was measured at.

## The matching distance was too tight

The reference pipeline does not use a flat correspondence distance at all: it
is `2.0 * ADAPTIVE_THRESHOLD_SIGMA`, ranging roughly 1.0 to 4.0 m under a
quality controller. Ours was a flat 0.4 m, chosen because a continuous-time
segment spans far less motion than a whole scan. Measured, that reasoning was
wrong about the *correspondence* distance, which has to cover the map's own
sampling and the local misregistration, not the motion within a segment:

| `match_threshold` | obsq-01 | obsq-02 | keble-02 | gt 10-01 | gt 11-02 |
|---|---|---|---|---|---|
| 0.2 | | 0.582 | | | |
| 0.4 | 0.0748 | 0.1595 | 0.0433 | 0.0615 | 1.178 |
| 0.8 | 0.0753 | 0.1302 | 0.0431 | 0.0521 | 1.323 |
| 1.5 | | 0.1301 | | | |

Better or unchanged on four of five, so 0.8 is the default. The exception is
the point-starved mission, which wants the window kept tight: with a median of
866 points per segment a wider window buys more wrong pairings than right
ones. That two sequences want opposite constants is the argument for the
adaptive threshold rather than a better guess at a fixed one.

What it should adapt *on* is not yet settled, and the obvious answer is
already ruled out. The reference adapts on registration quality, and the
natural analogue here is the LiDAR block's reduced chi-square, which we
already compute. It does not discriminate: grand-tour 2024-10-01 sits at
0.00301 and 2024-11-02 at 0.00321, near-identical, while wanting opposite
windows. The only quantity that separates them is the absolute point count,
896 per segment against 7400. Density, not residual, is the candidate signal,
on one data point; the remaining missions are what test it.

## Map and source resolution are not the bottleneck

Worth recording as a dead end. The reference method decimates to 0.10 m for
matching and 0.15 m for the map against this package's 0.4 m, which looked
like an obvious explanation for a 20x drift gap. It is not: on obsq-01,
0.4/0.4 gives 1.456 m, 0.2/0.2 gives 1.607 m, 0.15/0.15 gives 1.668 m and
0.10/0.15 gives 1.492 m. Everything lands within noise of everything else, at
several times the cost. Whatever the remaining gap is, it is structural rather
than a sampling density.

## The LiDAR/IMU weighting is not a constant, and must stop being one

This is the largest remaining effect, and the sweep now says something sharper
than "tune it". Full-sequence APE against `accel_noise_density`:

| accel_noise_density | obsq-01 | obsq-02 | keble-02 |
|---|---|---|---|
| 2e-3 (datasheet) | 1029 | | |
| 1e-2 | 66.1 | | |
| 5e-2 (default) | 1.456 | 1.546 | 1.153 |
| 2e-1 | 1.396 | **853** | **0.459** |
| 5e-1 | 1.272 | | |
| 1.0 | 1.227 | | |

Read the 2e-1 row. The same value that more than halves keble-02's error makes
obsq-02 diverge outright, on the same sensor, the same rig and the same day.
There is no scalar that is right for all three, so no amount of further
sweeping produces a defensible default: the 1.227 at the bottom of the obsq-01
column is not a result, it is a number that would blow up somewhere else.

The cause is the one already described: the inertial factors are whitened by a
genuine noise covariance while GICP's `cov_inv` is a surface-shape matrix
whose absolute scale has no relation to a measurement noise, and that scale
moves with the scene. `accel_noise_density` is being used to cancel a quantity
that is not constant, which is why it cannot be.

The fix is to stop hand-balancing and let the residuals set the ratio. The
LiDAR block's reduced chi-square says directly how far its information
matrices are from describing the sensor's noise, and on obsq-01 it is 0.0107,
tightly held (p10 0.0084, p90 0.0171). The residuals are about 93 times
smaller than the covariances claim, so the block is that much *more* precise
than it says, and `accel_noise_density` was being raised to compensate.

Note the direction: the conventional Birge ratio is `max(1, chi2/dof)`, which
only ever scales an over-confident block *down* and would be entirely inert
here. `lidar_balance: TwoSided` drops the clamp so an under-confident block is
corrected too, which is the case this data presents.

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

## A note on editing this file's C++ by pattern match

`clang-format` rewraps argument lists, so a patch that matches on a multi-line
call can silently fail to apply while a matching change to the format string
succeeds. That happened here: the state dump grew a `%.4f` for `imuCoverage`
whose argument was never passed, so the column read zero on every run,
including healthy ones, and briefly looked like evidence that no segment had
any inertial coverage at all. A printf conversion with no argument is
undefined behavior, not merely a wrong number.

If a diagnostic reads exactly zero everywhere, including where it cannot be,
check that it is being passed before believing it.

## What the residual actually tracks

The per-window reduced chi-square of the LiDAR block is **motion-driven**, not
density-driven. Across four missions its logarithm correlates +0.48 to +0.72
with speed and +0.46 to +0.68 with acceleration, while segment point count and
inlier count correlate |r| <= 0.36 and usually under 0.1.

In the worst 2.5% of windows the correspondence count, the segment density and
the inlier ratio all stay within 4% of normal, so a bad window is not a sparse
one.

Read `lidarPosInfo` with care: it is dumped *after* being multiplied by the
balance, so at a window where the balance collapsed it understates the
geometry's own contribution by exactly that factor. Divide the balance back
out before comparing windows. Doing so, the raw information per correspondence
at arc-3's spike windows is 1.16x its normal value, i.e. unchanged, and
`lidarCond` is 1.38x, i.e. slightly better conditioned. Neither the amount of
geometric information nor its distribution across the three axes degrades at a
spike. Only the residual rises.

This matters because the balance reacts to that chi-square. Down-weighting the
whole LiDAR block during fast motion is only correct if the geometry really is
worse there; if the residual grew because the prediction did, it is backwards.
`lidarCond` and `lidarWeakest` in the state dump exist to separate the two:
`lidarCond` is the smallest-over-largest eigenvalue of the window's summed
translational information, so a view that pins position only within a plane
collapses it while leaving the correspondence count untouched. Measured, it
does not collapse, which rules out geometric degeneracy as the cause and
leaves the residual itself as the only thing that moves.

## The balance's ceiling and degenerate geometry

A rank-deficient view fits well along the directions it does constrain, so its
residuals are small, so a residual-driven balance concludes the LiDAR is
precise and awards it the maximum weight. That weight arrives in every
direction, including the one the geometry says nothing about, and what was
holding that direction is the inertial term.

Measured on the shipped configuration: on thirteen of fourteen grand-tour
missions `log(lidarScale)` is *negatively* correlated with `log(lidarCond)`,
reaching -0.57 on eth-1 and -0.53 on con-4, and the windows that reach the
1000x ceiling are six to nine times worse conditioned than that mission's own
median. Oxford never reaches the ceiling at all, on any of three sequences,
because its clouds carry real per-point times and a segment spans a real alpha
range, so degenerate views are far rarer.

`lidar_balance_conditioning_reference` makes the ceiling proportional to the
conditioning instead of constant. Ships disabled.

Note which tail this lives in. The high-residual windows are *better*
conditioned than average, because well-conditioned geometry is what makes
misfit show up as residual at all. Looking at residual spikes will not find
this; it is the quiet windows that are dangerous.
