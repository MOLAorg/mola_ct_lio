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
 * @file   CtSegment.h
 * @brief  Continuous-time pose interpolation between two control knots
 */
#pragma once

#include <mola_ct_lio/se3.h>

namespace mola::ct
{
using Mat3x6 = Eigen::Matrix<double, 3, 6>;
using Mat3x12 = Eigen::Matrix<double, 3, 12>;
using Mat6x12 = Eigen::Matrix<double, 6, 12>;
using Mat12 = Eigen::Matrix<double, 12, 12>;
using Vec12 = Eigen::Matrix<double, 12, 1>;

/** The trajectory over one segment, i.e. between the two control knots that
 * bracket it, together with the Jacobians of an interpolated pose with respect
 * to both knots.
 *
 * The parameterization is the one from Traj-LO: a constant body-frame twist,
 * taken in the decoupled SE(3) form, so that for a point observed at
 * normalized time `alpha` in [0,1],
 *
 *     w     = Log(R_b^T R_e)
 *     R(a)  = R_b * Exp(a * w)
 *     p(a)  = (1-a) * p_b + a * p_e
 *
 * The rotation slerps between the knots while the position blends linearly in
 * the world frame. Both knots are free variables of the window optimization,
 * which is what makes the trajectory continuous rather than one pose per scan.
 */
class CtSegment
{
public:
  CtSegment() = default;

  /** @param T_begin Pose of the knot at the start of the segment (world<-body).
   *  @param T_end   Pose of the knot at the end of the segment.
   */
  CtSegment(const SE3 & T_begin, const SE3 & T_end) { setKnots(T_begin, T_end); }

  void setKnots(const SE3 & T_begin, const SE3 & T_end);

  /** Interpolated pose at normalized time `alpha` in [0,1]. */
  [[nodiscard]] SE3 poseAt(double alpha) const;

  /** Jacobian of a body-frame increment of the interpolated pose at `alpha`
   * with respect to the increments of both knots, stacked as
   * `[begin(6) ; end(6)]`.
   *
   * The increment convention is the one of incPose(): world-frame translation,
   * body-frame rotation. The returned block maps those onto a *body-frame*
   * increment of the interpolated pose, which is what pointJacobian() expects.
   *
   * Evaluating this at a linearization point different from the current knots
   * is what keeps a marginalized state's contribution consistent, so the
   * quantities it needs are taken from `linearizationKnots()` rather than
   * recomputed per call.
   */
  [[nodiscard]] Mat6x12 interpolationJacobian(double alpha) const;

  /** Jacobian of a world point with respect to a body-frame increment of the
   * pose that placed it there, for a point given in the body frame.
   */
  [[nodiscard]] static Mat3x6 pointJacobian(const SE3 & T_w_i, const Vec3 & pointInBody);

  [[nodiscard]] const SE3 & begin() const { return T_begin_; }
  [[nodiscard]] const SE3 & end() const { return T_end_; }

  /** The constant twist over the segment, `[u ; w]` in the decoupled form. */
  [[nodiscard]] const Vec6 & twist() const { return twist_; }

private:
  SE3 T_begin_;
  SE3 T_end_;
  Vec6 twist_ = Vec6::Zero();

  // Cached because they are shared by every point of the segment:
  Mat3 R_begin_t_ = Mat3::Identity();
  Mat3 R_end_t_ = Mat3::Identity();
  Mat3 R_end_begin_ = Mat3::Identity();
  Mat3 jacobianInvOmega_ = Mat3::Identity();
};

}  // namespace mola::ct
