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
 * @file   ImuFactor.h
 * @brief  Preintegrated IMU factor between two consecutive control knots
 */
#pragma once

#include <mola_ct_lio/se3.h>

namespace mola::ct
{
/** Dimension of one control knot's state when the IMU is in use. */
constexpr int kKnotDim = 15;

/** Offsets of each part within a knot's state block. The pose part keeps the
 * translation-then-rotation ordering of the LiDAR blocks, so a pose-only
 * 12x12 contribution drops into the window system without reshuffling.
 */
constexpr int kIdxPosition = 0;
constexpr int kIdxRotation = 3;
constexpr int kIdxVelocity = 6;
constexpr int kIdxBiasAcc = 9;
constexpr int kIdxBiasGyro = 12;

using Mat9 = Eigen::Matrix<double, 9, 9>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat9x30 = Eigen::Matrix<double, 9, 30>;
using Mat30 = Eigen::Matrix<double, 30, 30>;
using Vec30 = Eigen::Matrix<double, 30, 1>;

/** The outcome of preintegrating the IMU samples that fall between two knots.
 *
 * This mirrors `mola::imu::PreintegratedImuMeasurements` in plain Eigen, so
 * that the estimator core stays buildable and testable without the rest of the
 * MOLA stack. The conversion happens at the module boundary.
 *
 * Deltas follow Forster et al. (2017), expressed in the body frame of knot i:
 *
 *     dR_ij = R_i^T R_j
 *     dV_ij = R_i^T (v_j - v_i - g * dt)
 *     dP_ij = R_i^T (p_j - p_i - v_i * dt - 0.5 * g * dt^2)
 *
 * so gravity enters only here, when the delta meets a pair of states, never
 * during the integration itself.
 */
struct PreintegratedImu
{
  double dt = 0;

  Mat3 dR = Mat3::Identity();
  Vec3 dV = Vec3::Zero();
  Vec3 dP = Vec3::Zero();

  /// First-order bias-update Jacobians, about the linearization point below.
  Mat3 dR_dbg = Mat3::Zero();
  Mat3 dV_dba = Mat3::Zero();
  Mat3 dV_dbg = Mat3::Zero();
  Mat3 dP_dba = Mat3::Zero();
  Mat3 dP_dbg = Mat3::Zero();

  /// Propagated noise covariance, ordered `[theta, v, p]`.
  Mat9 cov = Mat9::Identity();

  /// The bias the deltas and Jacobians above were linearized about.
  Vec3 biasAcc = Vec3::Zero();
  Vec3 biasGyro = Vec3::Zero();
};

/** The navigation state carried by one control knot. */
struct KnotState
{
  SE3 T;
  Vec3 v = Vec3::Zero();
  Vec3 biasAcc = Vec3::Zero();
  Vec3 biasGyro = Vec3::Zero();
};

using Vec15 = Eigen::Matrix<double, 15, 1>;

/** Applies a 15-vector increment to a knot, in the convention shared by every
 * Jacobian here: world-frame translation and velocity, body-frame rotation,
 * plain additive biases.
 */
inline void incKnot(KnotState & s, const Vec15 & inc)
{
  incPose(s.T, inc.head<6>());
  s.v += inc.segment<3>(kIdxVelocity);
  s.biasAcc += inc.segment<3>(kIdxBiasAcc);
  s.biasGyro += inc.segment<3>(kIdxBiasGyro);
}

/** Residual of the preintegrated IMU factor, ordered `[theta, v, p]`. */
[[nodiscard]] Vec9 imuResidual(
  const PreintegratedImu & pim, const KnotState & si, const KnotState & sj, const Vec3 & gravity);

/** Residual and its Jacobian with respect to the stacked increments of both
 * knots, `[knot_i(15) ; knot_j(15)]`, in the increment convention of
 * incPose(): world-frame translation and velocity, body-frame rotation.
 */
[[nodiscard]] Vec9 imuResidualAndJacobian(
  const PreintegratedImu & pim, const KnotState & si, const KnotState & sj, const Vec3 & gravity,
  Mat9x30 & J);

/** Normal-equation contribution of one IMU factor, whitened by `pim.cov`. */
/** A single knot's contribution, over its full state. */
struct KnotBlock
{
  Eigen::Matrix<double, kKnotDim, kKnotDim> H = Eigen::Matrix<double, kKnotDim, kKnotDim>::Zero();
  Vec15 g = Vec15::Zero();
  double chi2 = 0;
};

struct ImuBlock
{
  Mat30 H = Mat30::Zero();
  Vec30 g = Vec30::Zero();
  double chi2 = 0;
};

[[nodiscard]] ImuBlock assembleImuBlock(
  const PreintegratedImu & pim, const KnotState & si, const KnotState & sj, const Vec3 & gravity);

/** Normal-equation contribution of the bias random walk between two knots.
 *
 * @param sigmaAcc  Accelerometer bias random-walk density [m/s^2/sqrt(s)].
 * @param sigmaGyro Gyroscope bias random-walk density [rad/s/sqrt(s)].
 */
[[nodiscard]] ImuBlock assembleBiasRandomWalkBlock(
  const KnotState & si, const KnotState & sj, double dt, double sigmaAcc, double sigmaGyro);

/** Keeps the IMU bias states inside the range a real sensor can have.
 *
 * The random-walk factor only ever speaks about the *difference* between two
 * consecutive knots' biases, so the biases themselves are free to wander as
 * far as the rest of the system pushes them, and a bias of several g is
 * reachable without any factor objecting. That is not a bias any more: it is
 * the estimator explaining a registration failure with the one state cheap
 * enough to absorb it.
 *
 * This adds the missing absolute statement. The sigmas are meant to be loose
 * enough that a genuine bias never feels them, so this changes nothing in
 * normal operation and only bites when the estimate leaves physical reality.
 * A zero sigma disables its half of the term.
 */
[[nodiscard]] KnotBlock assembleBiasPriorBlock(
  const KnotState & state, double sigmaAcc, double sigmaGyro);

}  // namespace mola::ct
