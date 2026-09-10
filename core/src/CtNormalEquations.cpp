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
 * @file   CtNormalEquations.cpp
 * @brief  LiDAR residuals of one segment, accumulated onto its two knots
 */
#include <mola_ct_lio/CtNormalEquations.h>

#include <cmath>

namespace mola::ct
{
namespace
{
double robustWeight(RobustKernel kernel, double scale, double residualSquaredNorm)
{
  switch (kernel) {
    case RobustKernel::GemanMcClure: {
      const double s2 = scale * scale;
      const double d = s2 + residualSquaredNorm;
      return (s2 * s2) / (d * d);
    }
    case RobustKernel::Cauchy: {
      // The form used by Traj-LO, whose "kinematic" threshold is this scale.
      const double s2 = scale * scale;
      return s2 / (s2 + residualSquaredNorm);
    }
    case RobustKernel::None:
    default:
      return 1.0;
  }
}
}  // namespace

LidarBlock assembleSegmentBlock(
  const CtSegment & segment, const CtSegment & jacobianAt, const std::vector<SegmentPoint> & points,
  const std::vector<PointCorrespondence> & correspondences, RobustKernel kernel, double kernelScale)
{
  LidarBlock out;

  for (const auto & c : correspondences) {
    if (c.localIndex >= points.size()) {
      continue;
    }
    const SegmentPoint & sp = points[c.localIndex];

    // The residual always uses the current estimate, so that the linear system
    // is solved about where the trajectory actually is:
    const SE3 T_w_i = segment.poseAt(sp.alpha);
    const Vec3 residual = c.globalPoint - (T_w_i * sp.p);

    const double weight = robustWeight(kernel, kernelScale, residual.squaredNorm());

    // The Jacobian may be taken elsewhere, see the header:
    const SE3 T_w_i_jac = jacobianAt.poseAt(sp.alpha);
    const Mat3x12 J =
      CtSegment::pointJacobian(T_w_i_jac, sp.p) * jacobianAt.interpolationJacobian(sp.alpha);

    const Eigen::Matrix<double, 12, 3> JtOmega = J.transpose() * c.information;

    out.H.noalias() += weight * JtOmega * J;
    out.g.noalias() += weight * JtOmega * residual;
    out.chi2 += weight * residual.dot(c.information * residual);
    out.errorSum += residual.norm();
    out.inliers++;
  }

  return out;
}

}  // namespace mola::ct
