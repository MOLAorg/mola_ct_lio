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

#if defined(MOLA_CT_LIO_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#endif

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

/** Accumulates one correspondence. Shared by the serial and the parallel
 * paths so they cannot drift apart.
 */
void accumulateOne(
  const CtSegment & segment, const CtSegment & jacobianAt, const std::vector<SegmentPoint> & points,
  const PointCorrespondence & c, RobustKernel kernel, double kernelScale, LidarBlock & out)
{
  if (c.localIndex >= points.size()) {
    return;
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

LidarBlock operator+(const LidarBlock & a, const LidarBlock & b)
{
  LidarBlock out;
  out.H = a.H + b.H;
  out.g = a.g + b.g;
  out.chi2 = a.chi2 + b.chi2;
  out.errorSum = a.errorSum + b.errorSum;
  out.inliers = a.inliers + b.inliers;
  return out;
}

/** Correspondences per parallel chunk.
 *
 * Fixed on purpose. The deterministic reduction fixes the split points and the
 * reduction tree from the range and this grain alone, so the floating-point
 * summation order does not depend on the thread count or on task stealing, and
 * a run stays bit-exact. An automatic grain would give that up.
 */
constexpr std::size_t kGrainSize = 512;

}  // namespace

LidarBlock assembleSegmentBlock(
  const CtSegment & segment, const CtSegment & jacobianAt, const std::vector<SegmentPoint> & points,
  const std::vector<PointCorrespondence> & correspondences, RobustKernel kernel, double kernelScale)
{
#if defined(MOLA_CT_LIO_HAS_TBB)
  return tbb::parallel_deterministic_reduce(
    tbb::blocked_range<std::size_t>(0, correspondences.size(), kGrainSize), LidarBlock(),
    [&](const tbb::blocked_range<std::size_t> & r, LidarBlock acc) {
      for (std::size_t i = r.begin(); i != r.end(); i++) {
        accumulateOne(segment, jacobianAt, points, correspondences[i], kernel, kernelScale, acc);
      }
      return acc;
    },
    [](const LidarBlock & a, const LidarBlock & b) { return a + b; });
#else
  LidarBlock out;
  for (const auto & c : correspondences) {
    accumulateOne(segment, jacobianAt, points, c, kernel, kernelScale, out);
  }
  return out;
#endif
}

}  // namespace mola::ct
