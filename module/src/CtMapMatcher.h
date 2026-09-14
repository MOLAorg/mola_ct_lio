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
#include <string>
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

  /** Which point of an occupied voxel stands for the voxel after decimation.
   *
   * The names match `mp2p_icp_filters::DecimateMethod` where the rule is the
   * same one, so that a result measured here transfers upstream.
   */
  enum class DecimateMethod
  {
    /// The first point reaching the voxel, i.e. the earliest in scan order.
    FirstPoint,

    /// The k-th point of the k-th occupied voxel, modulo how many it holds.
    /// Whatever scan order does to the choice is then spread across the
    /// voxels instead of applying to all of them the same way.
    RotatingIndex,

    /// The mean of the voxel's points. Not one of the input points.
    Centroid,

    /// The input point nearest the voxel's geometric center.
    ClosestToCenter,

    /// The input point nearest the mean of the voxel's points.
    ClosestToAverage,

    /// One input point picked by hashing the voxel's own coordinates, so the
    /// choice is arbitrary with respect to scan order yet reproducible.
    RandomPoint
  };

  struct Params
  {
    /// Voxel size used to decimate a segment's points before matching. [m]
    double sourceVoxelSize = 0.4;

    /// Keep one occupied source voxel in this many. Thins the cloud without
    /// coarsening the points that remain, which raising the voxel size would.
    /// One keeps every voxel.
    int sourceVoxelStride = 1;

    /// Fewest points a segment should carry into the optimization before the
    /// voxel is refined to find more.
    ///
    /// A fixed cell cannot serve both ends of this corpus: measured across
    /// every sequence, refining it is neutral or harmful above roughly 2500
    /// points per segment and worth a factor of five below about 1700. Since
    /// the harm is confined to the dense end, the rule is a floor rather than
    /// a target: a segment already above it is decimated exactly as before,
    /// and only a short one pays for a second pass. Zero disables.
    std::size_t minSegmentPoints = 2500;

    /// Voxel size used to decimate points on their way into the map. [m]
    double mapVoxelSize = 0.4;

    /// A candidate point closer than this to a point already in the map is
    /// not inserted. Zero disables the test, which inserts every decimated
    /// point of every segment.
    ///
    /// Insertion is driven by the segment rate, so how many copies of one
    /// surface reach the map is set by how long the platform spent looking at
    /// it. The copies do not describe the surface any better: they differ by
    /// the pose error of the window each arrived in, so the neighborhood the
    /// GICP covariances are fitted to ends up describing that error. Rejecting
    /// a candidate that lands on a point already held keeps the map's density
    /// a property of the geometry while still admitting whatever is new. [m]
    double mapMinPointSeparation = 0.0;

    /// Rejects a pairing whose combined GICP information is more isotropic
    /// than this, on a scale where one is perfectly scattered and zero is a
    /// perfect plane. Zero disables the test.
    ///
    /// The information matrix is fitted to a neighborhood, so it describes a
    /// surface only where there is one. Foliage and other volumetric clutter
    /// return a near-isotropic matrix: the fit is not wrong about a plane, it
    /// is a plane fitted to something that is not planar, and the pairing it
    /// weights pulls in a direction the geometry never constrained. Neither
    /// the correspondence distance nor the window's conditioning can see this,
    /// the first because such a pairing is genuinely nearby and the second
    /// because a window can be well conditioned overall while a fraction of
    /// its pairings sit in clutter.
    ///
    /// Measured as `3*det^(1/3)/trace`, which is one for an isotropic matrix
    /// and falls toward zero as it becomes anisotropic, and which costs a
    /// determinant and a trace rather than an eigendecomposition.
    double matchMaxIsotropy = 0.0;

    /// How an occupied voxel's representative point is chosen, for both the
    /// source cloud and the map.
    DecimateMethod decimateMethod = DecimateMethod::FirstPoint;

    /// Let the map rebuild its search tree on a background thread.
    ///
    /// Off by default, and deliberately: a rebuild in flight makes a query's
    /// answer depend on how far it has got, so two runs over the same bag
    /// stop agreeing bit for bit. That is worth paying for a result that has
    /// to be defended, and worth switching off when the question is how fast
    /// the method can go, since map insertion dominates the profile.
    bool mapAsyncRebuild = false;

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

  /** Keeps one point per occupied voxel, chosen by `method`.
   *
   * Deterministic by construction: the voxel keys are visited in a total
   * order fixed by the geometry, every rule reads only the point data, and
   * the survivors come back in the input's own order.
   */
  [[nodiscard]] static std::vector<ct::SegmentPoint> downsample(
    const std::vector<ct::SegmentPoint> & in, double voxelSize, int stride = 1,
    DecimateMethod method = DecimateMethod::FirstPoint);

  /** How far a decimation rule's chosen points sit from their voxels' means.
   *
   * Diagnostic only: a rule whose mean offset is not zero moves the cloud
   * bodily, and if the direction holds in the body frame it turns into a
   * heading-dependent error rather than noise.
   */
  struct DecimationBias
  {
    std::size_t voxels = 0;
    std::size_t points = 0;

    /// Mean over voxels of (chosen point - voxel mean), in the input frame.
    ct::Vec3 meanOffset = ct::Vec3::Zero();

    /// Root mean square of the same offset's length.
    double rmsOffset = 0;
  };

  [[nodiscard]] static DecimationBias measureBias(
    const std::vector<ct::SegmentPoint> & in, double voxelSize, DecimateMethod method);

  /** Parses the YAML spelling of a decimation method. */
  [[nodiscard]] static DecimateMethod decimateMethodFromString(const std::string & s);

  /** Correspondence query for one segment, in the form the estimator wants. */
  /** How the pairings of one match call sit along the sensor's own view rays.
   *
   * The map is searched by proximity alone, so a scan point may pair with a
   * map point on the far side of a thin structure, or across an occlusion
   * boundary. Such a pairing is geometrically wrong while its residual stays
   * small, so no residual-based rule can see it. Comparing each pairing's two
   * ranges from the same sensor origin can: a map point materially farther
   * along the ray than the point that was actually observed is behind a
   * surface the sensor saw, and could not have produced that return.
   */
  struct ViewRayStats
  {
    std::size_t pairings = 0;
    /// Map point farther along the ray than the observed point, beyond the
    /// tolerance; i.e. the pairing reaches through the observed surface.
    std::size_t behind = 0;
    /// Map point nearer along the ray, beyond the tolerance.
    std::size_t inFront = 0;
    /// Mean of |rangeMap - rangeScan| over all pairings [m].
    double meanRangeGap = 0;
  };

  /// Statistics of the most recent match() call. Costs two square roots per
  /// pairing against a nearest-neighbor search, so it is always collected.
  [[nodiscard]] const ViewRayStats & lastViewRayStats() const { return viewRayStats_; }

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
  mutable ViewRayStats viewRayStats_;
  std::size_t lastInsertedPoints_ = 0;
  mutable std::size_t lastIsotropyRejected_ = 0;

public:
  /// Points actually written by the most recent insert() call.
  [[nodiscard]] std::size_t lastInsertedPoints() const { return lastInsertedPoints_; }

  /// Pairings dropped by the isotropy test in the most recent match().
  [[nodiscard]] std::size_t lastIsotropyRejected() const { return lastIsotropyRejected_; }

private:
  uint32_t insertionsSincePrune_ = 0;

  void applyCovarianceOptions(IncrementalPointCloud & m) const;
};

}  // namespace mola
