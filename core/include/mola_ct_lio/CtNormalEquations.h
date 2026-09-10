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
 * @file   CtNormalEquations.h
 * @brief  LiDAR residuals of one segment, accumulated onto its two knots
 */
#pragma once

#include <mola_ct_lio/CtSegment.h>

#include <cstdint>
#include <vector>

namespace mola::ct
{
/** A source point of one segment, in the body frame of the segment's begin
 * knot, together with its normalized time within the segment.
 */
struct SegmentPoint
{
  Vec3 p = Vec3::Zero();
  double alpha = 0;
};

/** One accepted correspondence between a segment point and the map.
 *
 * `information` is what makes this generic over the residual type: it is the
 * matcher's own weighting of a world-frame 3-vector residual, so
 * point-to-point contributes a scaled identity, point-to-plane the rank-one
 * `n n^T`, and GICP the full `(C_global + R C_local R^T)^-1`. Nothing else in
 * the assembly changes between them.
 */
struct PointCorrespondence
{
  uint32_t localIndex = 0;
  Vec3 globalPoint = Vec3::Zero();
  Mat3 information = Mat3::Identity();
};

/** M-estimator applied on top of whatever the matcher already weighted. */
enum class RobustKernel : uint8_t
{
  None,
  GemanMcClure,
  Cauchy
};

/** The contribution of one segment to the window's normal equations, in the
 * stacked ordering `[begin knot(6) ; end knot(6)]`.
 */
struct LidarBlock
{
  Mat12 H = Mat12::Zero();
  Vec12 g = Vec12::Zero();
  double chi2 = 0;
  double errorSum = 0;
  std::size_t inliers = 0;
};

/** Accumulates the LiDAR residuals of one segment onto its two control knots.
 *
 * @param segment       Trajectory used to place each point, i.e. the current estimate.
 * @param jacobianAt    Trajectory the Jacobians are taken at. Pass the same as
 *                      `segment` in the ordinary case; a knot that has already
 *                      been marginalized must keep contributing at the fixed
 *                      linearization point its prior was built at, or the prior
 *                      and the new data stop describing the same quantity.
 * @param points        Source points of the segment, indexed by
 *                      PointCorrespondence::localIndex.
 * @param correspondences Accepted pairings, in a fixed order. The order decides
 *                      the floating-point summation order, so it must be
 *                      reproducible for the result to be.
 * @param kernelScale   Kernel scale in meters; ignored for RobustKernel::None.
 */
[[nodiscard]] LidarBlock assembleSegmentBlock(
  const CtSegment & segment, const CtSegment & jacobianAt, const std::vector<SegmentPoint> & points,
  const std::vector<PointCorrespondence> & correspondences, RobustKernel kernel,
  double kernelScale);

}  // namespace mola::ct
