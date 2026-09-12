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
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

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

/** Mean of the points a voxel holds, summed in index order so that two runs
 * over the same input agree bit for bit. */
ct::Vec3 voxelMean(const std::vector<ct::SegmentPoint> & in, const std::vector<std::size_t> & idx)
{
  ct::Vec3 sum = ct::Vec3::Zero();
  for (const auto i : idx) {
    sum += in[i].p;
  }
  return sum / static_cast<double>(idx.size());
}

/** The index, within `idx`, of the point nearest `target`. Ties go to the
 * earliest candidate, so the answer does not depend on the visiting order. */
std::size_t nearestTo(
  const std::vector<ct::SegmentPoint> & in, const std::vector<std::size_t> & idx,
  const ct::Vec3 & target)
{
  std::size_t best = idx.front();
  double bestSqr = (in[best].p - target).squaredNorm();
  for (const auto i : idx) {
    const double d = (in[i].p - target).squaredNorm();
    if (d < bestSqr) {
      bestSqr = d;
      best = i;
    }
  }
  return best;
}

/** A well-mixed 64-bit hash, so that a voxel's own integer coordinates decide
 * which of its points is taken. Reproducible, and unrelated to scan order. */
uint64_t mixKey(const VoxelKey & key)
{
  auto mix = [](uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  };
  uint64_t h = mix(static_cast<uint64_t>(std::get<0>(key)));
  h = mix(h ^ static_cast<uint64_t>(std::get<1>(key)));
  h = mix(h ^ static_cast<uint64_t>(std::get<2>(key)));
  return h;
}

/** The chosen representative of one occupied voxel.
 *
 * `ordinal` is the voxel's rank in the deterministic key order, which the
 * rotating rule uses to walk the choice across neighboring voxels.
 */
ct::SegmentPoint pickRepresentative(
  const std::vector<ct::SegmentPoint> & in, const VoxelKey & key,
  const std::vector<std::size_t> & idx, std::size_t ordinal, double voxelSize,
  mola::CtMapMatcher::DecimateMethod method)
{
  using Method = mola::CtMapMatcher::DecimateMethod;

  switch (method) {
    case Method::FirstPoint:
      return in[idx.front()];

    case Method::RotatingIndex:
      return in[idx[ordinal % idx.size()]];

    case Method::Centroid: {
      ct::SegmentPoint out;
      out.p = voxelMean(in, idx);
      double alphaSum = 0;
      for (const auto i : idx) {
        alphaSum += in[i].alpha;
      }
      out.alpha = alphaSum / static_cast<double>(idx.size());
      return out;
    }

    case Method::ClosestToCenter: {
      const ct::Vec3 center(
        (static_cast<double>(std::get<0>(key)) + 0.5) * voxelSize,
        (static_cast<double>(std::get<1>(key)) + 0.5) * voxelSize,
        (static_cast<double>(std::get<2>(key)) + 0.5) * voxelSize);
      return in[nearestTo(in, idx, center)];
    }

    case Method::ClosestToAverage:
      return in[nearestTo(in, idx, voxelMean(in, idx))];

    case Method::RandomPoint:
      return in[idx[mixKey(key) % idx.size()]];
  }
  return in[idx.front()];
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
  if (cfg.has("decimate_method")) {
    params.decimateMethod = decimateMethodFromString(cfg["decimate_method"].as<std::string>());
  }
  if (cfg.has("map_async_rebuild")) {
    params.mapAsyncRebuild = cfg["map_async_rebuild"].as<bool>();
  }
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

  // See the parameter's documentation: off buys bit-for-bit repeatability,
  // on buys throughput, and map insertion is where the time goes.
  m.creationOptions.async_rebuild = params.mapAsyncRebuild;
}

std::vector<ct::SegmentPoint> CtMapMatcher::downsample(
  const std::vector<ct::SegmentPoint> & in, double voxelSize, int stride, DecimateMethod method)
{
  if (voxelSize <= 0) {
    return in;
  }

  // Every point of every occupied voxel, so that a rule can look at the
  // voxel's contents and not only at whichever point arrived first.
  std::map<VoxelKey, std::vector<std::size_t>> voxels;
  for (std::size_t i = 0; i < in.size(); i++) {
    voxels[voxelKeyOf(in[i].p, voxelSize)].push_back(i);
  }

  // Taking one occupied voxel in `stride` thins the cloud without coarsening
  // it: the points that survive keep their original spacing, where raising the
  // voxel size instead would move every one of them. The map key order is
  // deterministic, so which voxels survive is too.
  const int keepEvery = std::max(1, stride);

  // Each survivor is tagged with the index of the voxel's first point, so the
  // output can be put back into the input's order. That keeps a pairing's
  // index meaning what the caller expects and ties the summation order to the
  // input, for the rules whose representative is not an input point too.
  std::vector<std::pair<std::size_t, ct::SegmentPoint>> kept;
  kept.reserve(voxels.size() / static_cast<std::size_t>(keepEvery) + 1);

  std::size_t ordinal = 0;
  for (const auto & [key, indices] : voxels) {
    const std::size_t thisOrdinal = ordinal++;
    if (thisOrdinal % static_cast<std::size_t>(keepEvery) != 0) {
      continue;
    }
    kept.emplace_back(
      indices.front(), pickRepresentative(in, key, indices, thisOrdinal, voxelSize, method));
  }

  std::sort(
    kept.begin(), kept.end(), [](const auto & a, const auto & b) { return a.first < b.first; });

  std::vector<ct::SegmentPoint> out;
  out.reserve(kept.size());
  for (const auto & [index, point] : kept) {
    out.push_back(point);
  }
  return out;
}

CtMapMatcher::DecimationBias CtMapMatcher::measureBias(
  const std::vector<ct::SegmentPoint> & in, double voxelSize, DecimateMethod method)
{
  DecimationBias bias;
  if (voxelSize <= 0 || in.empty()) {
    return bias;
  }

  std::map<VoxelKey, std::vector<std::size_t>> voxels;
  for (std::size_t i = 0; i < in.size(); i++) {
    voxels[voxelKeyOf(in[i].p, voxelSize)].push_back(i);
  }

  ct::Vec3 sum = ct::Vec3::Zero();
  double sumSqr = 0;
  std::size_t ordinal = 0;
  for (const auto & [key, indices] : voxels) {
    const ct::Vec3 mean = voxelMean(in, indices);
    const auto rep = pickRepresentative(in, key, indices, ordinal++, voxelSize, method);
    const ct::Vec3 off = rep.p - mean;
    sum += off;
    sumSqr += off.squaredNorm();
  }

  bias.voxels = voxels.size();
  bias.points = in.size();
  bias.meanOffset = sum / static_cast<double>(voxels.size());
  bias.rmsOffset = std::sqrt(sumSqr / static_cast<double>(voxels.size()));
  return bias;
}

CtMapMatcher::DecimateMethod CtMapMatcher::decimateMethodFromString(const std::string & s)
{
  if (s == "FirstPoint") {
    return DecimateMethod::FirstPoint;
  }
  if (s == "RotatingIndex") {
    return DecimateMethod::RotatingIndex;
  }
  if (s == "Centroid") {
    return DecimateMethod::Centroid;
  }
  if (s == "ClosestToCenter") {
    return DecimateMethod::ClosestToCenter;
  }
  if (s == "ClosestToAverage") {
    return DecimateMethod::ClosestToAverage;
  }
  if (s == "RandomPoint") {
    return DecimateMethod::RandomPoint;
  }
  THROW_EXCEPTION_FMT("Unknown decimate_method: '%s'", s.c_str());
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

  // Tolerance on the range comparison below. Below roughly a voxel the two
  // ranges differ for reasons that carry no information: decimation, the
  // sensor's own noise, and the surface's thickness in the map.
  const double rayTolerance = 2.0 * params.mapVoxelSize;

  viewRayStats_ = ViewRayStats();
  double rangeGapSum = 0;

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

    // The point is stated in the sensor frame, so its own norm is the range
    // that produced it; the map point's range is taken from the same origin.
    const ct::SegmentPoint & sp = points[p.local_idx];
    const ct::Vec3 origin = segment.poseAt(sp.alpha).t;
    const double rangeScan = sp.p.norm();
    const double rangeMap = (c.globalPoint - origin).norm();
    const double gap = rangeMap - rangeScan;

    viewRayStats_.pairings++;
    rangeGapSum += std::abs(gap);
    if (gap > rayTolerance) {
      viewRayStats_.behind++;
    } else if (gap < -rayTolerance) {
      viewRayStats_.inFront++;
    }
  }
  if (viewRayStats_.pairings > 0) {
    viewRayStats_.meanRangeGap = rangeGapSum / static_cast<double>(viewRayStats_.pairings);
  }
}

void CtMapMatcher::insert(
  const ct::CtSegment & segment, const std::vector<ct::SegmentPoint> & points)
{
  if (points.empty()) {
    return;
  }

  const auto decimated = downsample(points, params.mapVoxelSize, 1, params.decimateMethod);
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
