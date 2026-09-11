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
 * @file   CtMapMatcher.h
 * @brief  The local map, and GICP matching of a segment against it
 */
#pragma once

#include <mola_ct_lio/CtNormalEquations.h>
#include <mola_metric_maps/IncrementalPointCloud.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/system/CTimeLogger.h>

#include <cstdint>
#include <vector>

namespace mola
{
/** Owns the odometry local map and answers the estimator's correspondence
 * queries against it.
 *
 * Matching is cov-to-cov, i.e. GICP: each source point is paired with its
 * nearest map point and weighted by `(C_global + R C_local R^T)^-1`. That
 * weighting reaches the estimator as a plain 3x3 information matrix, which is
 * the only thing the continuous-time assembly needs to know about the residual
 * type.
 *
 * The source points are deskewed into the world frame with the current
 * trajectory *before* matching, and the query is then made with an identity
 * pose. This is exact rather than an approximation: the local covariance
 * computed from world-frame neighbors is what `R C R^T` means for a segment
 * whose own motion is locally rigid.
 */
class CtMapMatcher
{
public:
  CtMapMatcher();
  ~CtMapMatcher();

  struct Params
  {
    /// Voxel size used to decimate a segment's points before matching. [m]
    double sourceVoxelSize = 0.4;

    /// Keep one occupied source voxel in this many. Thins the cloud without
    /// coarsening the points that remain, which raising the voxel size would.
    /// One keeps every voxel.
    int sourceVoxelStride = 1;

    /// Voxel size used to decimate points on their way into the map. [m]
    double mapVoxelSize = 0.4;

    /// Points farther than this from the latest sensor position are dropped. [m]
    double mapRadius = 120.0;

    /// How many insertions to make between two prunings of the map.
    ///
    /// Pruning rebuilds the k-d tree and recomputes every covariance, so doing
    /// it on every segment costs far more than the points it evicts are worth,
    /// and on a trajectory shorter than `mapRadius` it evicts nothing at all.
    /// The map is allowed to overshoot its radius by this many insertions.
    uint32_t prunePeriod = 25;

    /// Matching distance, and its optional range-adaptive form. [m]
    float matchThreshold = 0.8f;
    float matchThresholdFar = 0.0f;
    float matchKneeRange = 15.0f;
    float matchTransitionWidth = 5.0f;

    /// Per-point covariance estimation, on both the map and the source cloud.
    uint32_t kCov = 20;
    uint32_t minKCov = 5;
    double maxDistCov = 1.0;
    double maxPlaneDevCov = 0.0;
  };

  Params params;

  /// Shared with the engine's logger, so the breakdown is one table.
  mrpt::system::CTimeLogger * profiler = nullptr;

  void initialize(const mrpt::containers::yaml & cfg);

  /** Keeps one point per voxel, the first one seen.
   *
   * Deterministic by construction: the voxel keys are visited in a total
   * order that does not depend on the input order beyond which point is
   * first in each cell.
   */
  [[nodiscard]] static std::vector<ct::SegmentPoint> downsample(
    const std::vector<ct::SegmentPoint> & in, double voxelSize, int stride = 1);

  /** Correspondence query for one segment, in the form the estimator wants. */
  void match(
    const ct::CtSegment & segment, const std::vector<ct::SegmentPoint> & points,
    std::vector<ct::PointCorrespondence> & out) const;

  /** Adds a segment's points to the map, deskewed with the given trajectory. */
  void insert(const ct::CtSegment & segment, const std::vector<ct::SegmentPoint> & points);

  [[nodiscard]] bool empty() const;
  [[nodiscard]] std::size_t pointCount() const;

  [[nodiscard]] const IncrementalPointCloud & map() const { return *map_; }

private:
  std::shared_ptr<IncrementalPointCloud> map_;
  uint32_t insertionsSincePrune_ = 0;

  void applyCovarianceOptions(IncrementalPointCloud & m) const;
};

}  // namespace mola
