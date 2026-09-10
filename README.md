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

## Status

Early. See `core/` for the parts that exist and are tested.

## License

GPL-3.0. Traj-LO, whose trajectory mathematics this reuses, is MIT.
