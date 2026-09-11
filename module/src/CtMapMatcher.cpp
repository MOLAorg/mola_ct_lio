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
 * @file   CtMapMatcher.cpp
 * @brief  The local map, and GICP matching of a segment against it
 */
#include "CtMapMatcher.h"

#include <mrpt/core/exceptions.h>

#include <algorithm>
#include <map>
#include <optional>
#include <tuple>

namespace mola
{
namespace
{
using VoxelKey = std::tuple<int64_t, int64_t, int64_t>;

VoxelKey voxelKeyOf(const ct::Vec3 & p, double voxelSize)
{
  const double inv = 1.0 / voxelSize;
  return {
    static_cast<int64_t>(std::floor(p.x() * inv)), static_cast<int64_t>(std::floor(p.y() * inv)),
    static_cast<int64_t>(std::floor(p.z() * inv))};
}

/** The world-frame position of each source point, at its own timestamp. */
std::vector<ct::Vec3> deskew(
  const ct::CtSegment & segment, const std::vector<ct::SegmentPoint> & points)
{
  std::vector<ct::Vec3> out;
  out.reserve(points.size());
  for (const auto & sp : points) {
    out.push_back(segment.poseAt(sp.alpha) * sp.p);
  }
  return out;
}
}  // namespace

CtMapMatcher::CtMapMatcher() : map_(std::make_shared<IncrementalPointCloud>()) {}

CtMapMatcher::~CtMapMatcher() = default;

void CtMapMatcher::initialize(const mrpt::containers::yaml & cfg)
{
  const auto readDouble = [&](const char * key, double & target) {
    if (cfg.has(key)) {
      target = cfg[key].as<double>();
    }
  };
  const auto readFloat = [&](const char * key, float & target) {
    if (cfg.has(key)) {
      target = cfg[key].as<float>();
    }
  };
  const auto readUint = [&](const char * key, uint32_t & target) {
    if (cfg.has(key)) {
      target = cfg[key].as<uint32_t>();
    }
  };

  readDouble("source_voxel_size", params.sourceVoxelSize);
  if (cfg.has("source_voxel_stride")) {
    params.sourceVoxelStride = cfg["source_voxel_stride"].as<int>();
  }
  if (cfg.has("min_segment_points")) {
    params.minSegmentPoints = cfg["min_segment_points"].as<std::size_t>();
  }
  readDouble("map_voxel_size", params.mapVoxelSize);
  readDouble("map_radius", params.mapRadius);
  readUint("map_prune_period", params.prunePeriod);
  readFloat("match_threshold", params.matchThreshold);
  readFloat("match_threshold_far", params.matchThresholdFar);
  readFloat("match_knee_range", params.matchKneeRange);
  readFloat("match_transition_width", params.matchTransitionWidth);
  readUint("k_correspondences_for_cov", params.kCov);
  readUint("min_correspondences_for_cov", params.minKCov);
  readDouble("max_distance_for_cov", params.maxDistCov);
  readDouble("max_plane_deviation_for_cov", params.maxPlaneDevCov);

  map_ = std::make_shared<IncrementalPointCloud>();
  applyCovarianceOptions(*map_);
}

void CtMapMatcher::applyCovarianceOptions(IncrementalPointCloud & m) const
{
  m.creationOptions.k_correspondences_for_cov = params.kCov;
  m.creationOptions.min_correspondences_for_cov = params.minKCov;
  m.creationOptions.max_distance_for_cov = params.maxDistCov;
  m.creationOptions.max_plane_deviation_for_cov = params.maxPlaneDevCov;

  // A background rebuild would make a query's result depend on how far it had
  // got, which this estimator is not allowed to tolerate.
  m.creationOptions.async_rebuild = false;
}

std::vector<ct::SegmentPoint> CtMapMatcher::downsample(
  const std::vector<ct::SegmentPoint> & in, double voxelSize, int stride)
{
  if (voxelSize <= 0) {
    return in;
  }

  std::map<VoxelKey, std::size_t> firstInVoxel;
  for (std::size_t i = 0; i < in.size(); i++) {
    firstInVoxel.emplace(voxelKeyOf(in[i].p, voxelSize), i);
  }

  // Taking one occupied voxel in `stride` thins the cloud without coarsening
  // it: the points that survive keep their original spacing, where raising the
  // voxel size instead would move every one of them. The map key order is
  // deterministic, so which voxels survive is too.
  const int keepEvery = std::max(1, stride);

  std::vector<std::size_t> kept;
  kept.reserve(firstInVoxel.size() / static_cast<std::size_t>(keepEvery) + 1);
  std::size_t visited = 0;
  for (const auto & [key, index] : firstInVoxel) {
    if (visited++ % static_cast<std::size_t>(keepEvery) != 0) {
      continue;
    }
    kept.push_back(index);
  }
  // Back to the original order, so that a pairing's index still means what the
  // caller expects and the summation order stays tied to the input:
  std::sort(kept.begin(), kept.end());

  std::vector<ct::SegmentPoint> out;
  out.reserve(kept.size());
  for (const auto index : kept) {
    out.push_back(in[index]);
  }
  return out;
}

bool CtMapMatcher::empty() const { return map_->livePointCount() == 0; }

std::size_t CtMapMatcher::pointCount() const { return map_->livePointCount(); }

void CtMapMatcher::match(
  const ct::CtSegment & segment, const std::vector<ct::SegmentPoint> & points,
  std::vector<ct::PointCorrespondence> & out) const
{
  if (points.empty() || empty()) {
    return;
  }

  std::vector<ct::Vec3> world;
  {
    std::optional<mrpt::system::CTimeLoggerEntry> tle;
    if (profiler) tle.emplace(*profiler, "match.deskew");
    world = deskew(segment, points);
  }

  // The source cloud is handed over already in the world frame, so the query
  // pose is the identity and the local covariances are computed from the very
  // neighborhoods the residual will use.
  std::optional<mrpt::system::CTimeLoggerEntry> tleBuild;
  if (profiler) tleBuild.emplace(*profiler, "match.buildLocalMap");
  auto local = std::make_shared<IncrementalPointCloud>();
  applyCovarianceOptions(*local);
  local->creationOptions.reserve_points = world.size();
  local->reserve(world.size());
  for (const auto & p : world) {
    local->insertPoint(
      static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
  }

  if (tleBuild) tleBuild.reset();

  std::optional<mrpt::system::CTimeLoggerEntry> tleNN;
  if (profiler) tleNN.emplace(*profiler, "match.nnSearchCov2Cov");
  mp2p_icp::MatchedPointWithCovList pairings;

#if defined(MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE)
  mp2p_icp::MatchingDistanceProfile profile(params.matchThreshold);
  if (params.matchThresholdFar > 0) {
    profile.far = params.matchThresholdFar;
    profile.kneeRange = params.matchKneeRange;
    profile.width = params.matchTransitionWidth;
  }
  map_->nn_search_cov2cov(*local, mrpt::poses::CPose3D::Identity(), profile, pairings);
#else
  map_->nn_search_cov2cov(
    *local, mrpt::poses::CPose3D::Identity(), params.matchThreshold, pairings);
#endif

  if (tleNN) tleNN.reset();

  out.reserve(out.size() + pairings.size());
  for (const auto & p : pairings) {
    if (p.local_idx >= points.size()) {
      continue;
    }
    ct::PointCorrespondence c;
    c.localIndex = p.local_idx;
    c.globalPoint = ct::Vec3(p.global.x, p.global.y, p.global.z);
    c.information = p.cov_inv.asEigen().cast<double>();
    out.push_back(c);
  }
}

void CtMapMatcher::insert(
  const ct::CtSegment & segment, const std::vector<ct::SegmentPoint> & points)
{
  if (points.empty()) {
    return;
  }

  const auto decimated = downsample(points, params.mapVoxelSize);
  const std::vector<ct::Vec3> world = deskew(segment, decimated);

  map_->reserve(map_->size() + world.size());
  for (const auto & p : world) {
    map_->insertPoint(
      static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
  }

  // Eviction is driven explicitly from the segment's own end pose rather than
  // left to the map's insertion-time rule, so that what the map holds depends
  // only on the trajectory and not on insertion order.
  //
  // It is deliberately not done on every insertion: pruning rebuilds the k-d
  // tree and recomputes every covariance, which costs far more than the points
  // it drops are worth. The map simply carries up to `prunePeriod` segments of
  // overshoot beyond its radius.
  insertionsSincePrune_++;
  if (params.mapRadius > 0 && insertionsSincePrune_ >= std::max<uint32_t>(1, params.prunePeriod)) {
    insertionsSincePrune_ = 0;

    std::optional<mrpt::system::CTimeLoggerEntry> tle;
    if (profiler) tle.emplace(*profiler, "mapInsert.prune");

    const ct::Vec3 center = segment.end().t;
    map_->keepOnlyPointsNear(
      mrpt::math::TPoint3Df(
        static_cast<float>(center.x()), static_cast<float>(center.y()),
        static_cast<float>(center.z())),
      params.mapRadius);
  }
}

}  // namespace mola
