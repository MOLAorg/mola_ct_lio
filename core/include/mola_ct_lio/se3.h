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
 * @file   se3.h
 * @brief  Minimal SE(3) type and SO(3) helpers for the continuous-time core
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace mola::ct
{
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;

/** Skew-symmetric matrix of a 3-vector, such that `hat(a) * b == a.cross(b)`. */
inline Mat3 hat(const Vec3 & v)
{
  Mat3 m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/** SO(3) exponential map. */
inline Mat3 so3Exp(const Vec3 & phi)
{
  const double theta = phi.norm();
  const Mat3 K = hat(phi);
  if (theta < 1e-8) {
    // Second-order series; exact enough well below float resolution.
    return Mat3::Identity() + K + 0.5 * K * K;
  }
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  return Mat3::Identity() + (s / theta) * K + ((1.0 - c) / (theta * theta)) * K * K;
}

/** SO(3) logarithm map. */
inline Vec3 so3Log(const Mat3 & R)
{
  const Eigen::AngleAxisd aa(Eigen::Quaterniond(R).normalized());
  return aa.angle() * aa.axis();
}

/** Right Jacobian of SO(3), defined by `Exp(phi + d) ~= Exp(phi) * Exp(Jr(phi) * d)`.
 *
 * A Taylor expansion is used for a small angle, where the closed form loses
 * all its significant digits to cancellation.
 */
inline Mat3 rightJacobianSO3(const Vec3 & phi)
{
  const double theta2 = phi.squaredNorm();
  const Mat3 K = hat(phi);
  if (theta2 < 1e-12) {
    return Mat3::Identity() - 0.5 * K + (1.0 / 6.0) * K * K;
  }
  const double theta = std::sqrt(theta2);
  const double c1 = (1.0 - std::cos(theta)) / theta2;
  const double c2 = (theta - std::sin(theta)) / (theta2 * theta);
  return Mat3::Identity() - c1 * K + c2 * K * K;
}

/** Inverse of rightJacobianSO3(). */
inline Mat3 rightJacobianInvSO3(const Vec3 & phi)
{
  const double theta2 = phi.squaredNorm();
  const Mat3 K = hat(phi);
  if (theta2 < 1e-12) {
    return Mat3::Identity() + 0.5 * K + (1.0 / 12.0) * K * K;
  }
  const double theta = std::sqrt(theta2);
  const double c = (1.0 / theta2) - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
  return Mat3::Identity() + 0.5 * K + c * K * K;
}

/** A rigid transformation, stored as a rotation matrix plus a translation.
 *
 * The rotation is kept as a matrix rather than a quaternion because every hot
 * path here multiplies by it or by its transpose.
 */
struct SE3
{
  Mat3 R = Mat3::Identity();
  Vec3 t = Vec3::Zero();

  SE3() = default;
  SE3(const Mat3 & R_, const Vec3 & t_) : R(R_), t(t_) {}

  [[nodiscard]] SE3 inverse() const
  {
    const Mat3 Rt = R.transpose();
    return SE3(Rt, -(Rt * t));
  }

  [[nodiscard]] SE3 operator*(const SE3 & o) const { return SE3(R * o.R, R * o.t + t); }

  [[nodiscard]] Vec3 operator*(const Vec3 & p) const { return R * p + t; }

  /** Re-orthonormalizes the rotation, to stop round-off from accumulating over
   * a long sequence of increments.
   */
  void normalize() { R = Eigen::Quaterniond(R).normalized().toRotationMatrix(); }
};

/** The decoupled SE(3) logarithm used by the trajectory parameterization:
 * the translation is taken as-is, not mapped through the SE(3) left Jacobian.
 * Returns `[u ; w]`.
 */
inline Vec6 se3LogDecoupled(const SE3 & T)
{
  Vec6 v;
  v.head<3>() = T.t;
  v.tail<3>() = so3Log(T.R);
  return v;
}

/** Inverse of se3LogDecoupled(). */
inline SE3 se3ExpDecoupled(const Vec6 & v) { return SE3(so3Exp(v.tail<3>()), v.head<3>()); }

/** Applies a 6-vector increment to a pose.
 *
 * The convention, shared by the Jacobians and the solver, is a world-frame
 * translation increment and a body-frame (right) rotation increment:
 *
 *     p' = p + u,      R' = R * Exp(w)
 */
inline void incPose(SE3 & T, const Vec6 & inc)
{
  T.t += inc.head<3>();
  T.R = T.R * so3Exp(inc.tail<3>());
}

}  // namespace mola::ct
