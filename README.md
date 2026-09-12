# mola_ct_lio

Continuous-time LiDAR-inertial odometry for [MOLA](https://github.com/MOLAorg/mola).

The module interfaces are those of the other MOLA odometry front ends; the core
is what differs. Instead of estimating one pose per scan, it estimates a
**continuous trajectory** as a sliding window of control knots, and every point
is registered at its own timestamp against the map.

## What it is made of

- **Trajectory**: the constant-twist, decoupled SE(3) parameterization of
  [Traj-LO](https://github.com/kevin2431/Traj-LO) (Zheng & Zhu, RA-L 2024).
  Each scan is cut into fixed-length segments; consecutive control knots are
  optimized jointly. The mathematics is reused, the code is not.
- **Matching**: [`mp2p_icp`](https://github.com/MOLAorg/mp2p_icp) matchers,
  selected from YAML. The default is cov-to-cov (GICP) against
  `mola::IncrementalPointCloud`, an incremental k-d tree map with cached
  per-point covariances.
- **IMU**: tightly coupled Forster on-manifold preintegration factors
  (`mola::imu::ImuPreintegrator`), with per-knot velocity and bias states.
  LiDAR-only operation stays available and is the clean control arm.

The point of the design is that the interpolation Jacobian is independent of
the residual type: point-to-point, point-to-plane and GICP differ only in the
information matrix they contribute, so the matcher is a configuration choice
rather than something baked into the estimator.

## Results

Accuracy is measured against two public corpora, at the defaults the
pipelines below ship with, and with the reference composed onto the frame each
corpus anchors its ground truth to. `agents.md` records what each parameter is
worth and where the shipped value is not the best one measured.

**Oxford Spires**, absolute trajectory error [m]:

| sequence | this | best previously recorded here |
|---|---|---|
| keble-college-02 | **0.0433** | 0.0463 |
| observatory-quarter-01 | 0.0758 | 0.0641 |
| observatory-quarter-02 | 0.0615 | 0.0543 |

**GrandTour**, against the total-station prism reference [m]. The prism is
0.64 m from the body frame and the truth is position only, so the estimate has
to be carried out to that mount before scoring; `scripts/score-grandtour.py`
does it.

| mission | release folder | ATE |
|---|---|---|
| con-3 | 2024-12-09-11-28-28 | 0.0162 |
| heap-1 | 2024-11-14-13-45-37 | 0.0199 |
| con-1 | 2024-12-09-09-34-43 | 0.0232 |
| con-2 | 2024-12-09-09-41-46 | 0.0246 |
| eth-1 | 2024-10-01-11-29-55 | 0.0468 |

Two missions at one further site are left out of every figure above and out of
every decision behind them. Their error moves by an order of magnitude under
parameter changes that move the five above by under four percent, so a number
from either is one draw from a wide distribution rather than a measurement.
`agents.md` records how that was established.

## Usage

`mola-ct-lio-cli` replays a rosbag offline. Every parameter carries a
documented default and an environment override, all of them in the shared
`pipelines/common/ctlio-base.yaml`; each per-dataset file under `pipelines/`
imports that base and rebinds only the knobs its dataset actually changes:

```bash
mola-ct-lio-cli -c pipelines/ctlio-oxford-spires.yaml \
  --input-rosbag2 <bag> --lidar-topic <topic> --imu-topic <topic> \
  --output-tum-path out.tum
```

For a dataset with a wrapper there is nothing to spell out. Each one takes a
mission directory and drives either the offline batch or the same pipeline
replayed with the 3D GUI, from one shared description of the dataset:

```bash
mola-ct-lio-cli-grandtour /data/grand-tour/2024-10-01-11-29-55/
mola-ct-lio-gui-grandtour /data/grand-tour/2024-10-01-11-29-55/
```

Run either with no arguments to see what it can be told.

`agents.md` records what each parameter is worth, and which ideas were measured
and rejected.

## License

GPL-3.0. Traj-LO, whose trajectory mathematics this reuses, is MIT.
