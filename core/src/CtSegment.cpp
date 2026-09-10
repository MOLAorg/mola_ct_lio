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
 * @file   CtSegment.cpp
 * @brief  Continuous-time pose interpolation between two control knots
 */
#include <mola_ct_lio/CtSegment.h>

namespace mola::ct
{
void CtSegment::setKnots(const SE3 & T_begin, const SE3 & T_end)
{
  T_begin_ = T_begin;
  T_end_ = T_end;

  twist_ = se3LogDecoupled(T_begin_.inverse() * T_end_);

  R_begin_t_ = T_begin_.R.transpose();
  R_end_t_ = T_end_.R.transpose();
  R_end_begin_ = R_end_t_ * T_begin_.R;
  jacobianInvOmega_ = rightJacobianInvSO3(twist_.tail<3>());
}

SE3 CtSegment::poseAt(double alpha) const
{
  SE3 out;
  out.R = T_begin_.R * so3Exp(alpha * twist_.tail<3>());
  // The decoupled form makes the position a plain world-frame blend:
  out.t = T_begin_.t + alpha * (T_end_.t - T_begin_.t);
  return out;
}

Mat3x6 CtSegment::pointJacobian(const SE3 & T_w_i, const Vec3 & pointInBody)
{
  Mat3x6 J;
  J.block<3, 3>(0, 0) = T_w_i.R;
  J.block<3, 3>(0, 3) = -T_w_i.R * hat(pointInBody);
  return J;
}

Mat6x12 CtSegment::interpolationJacobian(double alpha) const
{
  const Vec3 omega = twist_.tail<3>();

  const Mat3 jacobianAlphaOmega = rightJacobianSO3(alpha * omega);
  const Mat3 chainedRotation = jacobianAlphaOmega * jacobianInvOmega_;
  const Mat3 rotationBack = so3Exp(-alpha * omega);

  Eigen::Matrix<double, 6, 6> J_begin = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> J_end = Eigen::Matrix<double, 6, 6>::Zero();

  // Translation: p(a) is a world-frame linear blend of the two knot positions,
  // so it does not depend on either rotation. R_i^T converts the resulting
  // world-frame increment into the body frame that pointJacobian() works in,
  // and it has two equivalent forms, one anchored on each knot.
  J_begin.topLeftCorner<3, 3>() = (1.0 - alpha) * rotationBack * R_begin_t_;
  J_end.topLeftCorner<3, 3>() = alpha * so3Exp((1.0 - alpha) * omega) * R_end_t_;

  // Rotation: perturbing either knot moves the twist, and the interpolated
  // rotation follows it through the right Jacobian of the partial rotation.
  J_begin.bottomRightCorner<3, 3>() = rotationBack - alpha * chainedRotation * R_end_begin_;
  J_end.bottomRightCorner<3, 3>() = alpha * chainedRotation;

  Mat6x12 J;
  J.block<6, 6>(0, 0) = J_begin;
  J.block<6, 6>(0, 6) = J_end;
  return J;
}

}  // namespace mola::ct
