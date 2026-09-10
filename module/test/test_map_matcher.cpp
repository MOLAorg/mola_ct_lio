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
 * @file   test_map_matcher.cpp
 * @brief  The estimator driven by the real GICP matcher, not a perfect one.
 *
 * Everything up to here fed the optimizer exact correspondences. This closes
 * the loop: a structured scene goes into the map, a segment is perturbed away
 * from where it was observed, and the trajectory has to come back using only
 * what the matcher finds.
 */
#include <gtest/gtest.h>
#include <mola_ct_lio/WindowOptimizer.h>

#include <random>

#include "CtMapMatcher.h"

namespace
{
using namespace mola;      // NOLINT(build/namespaces)
using namespace mola::ct;  // NOLINT(build/namespaces)

constexpr double kSegmentInterval = 0.04;

/** A corridor: two side walls, a floor and an end wall, which together leave
 * no direction unconstrained.
 */
std::vector<Vec3> makeScene()
{
  std::vector<Vec3> pts;
  for (double x = -5.0; x <= 25.0; x += 0.10) {
    for (double z = 0.0; z <= 3.0; z += 0.10) {
      pts.emplace_back(x, -4.0, z);
      pts.emplace_back(x, 4.0, z);
    }
    for (double y = -4.0; y <= 4.0; y += 0.10) {
      pts.emplace_back(x, y, 0.0);
    }
  }
  for (double y = -4.0; y <= 4.0; y += 0.10) {
    for (double z = 0.0; z <= 3.0; z += 0.10) {
      pts.emplace_back(25.0, y, z);
    }
  }
  return pts;
}

struct Truth
{
  Vec3 velocity{3.0, 0.0, 0.0};
  Vec3 angularVelocity{0.0, 0.0, 0.05};

  [[nodiscard]] SE3 at(double t) const
  {
    SE3 T;
    T.R = so3Exp(angularVelocity * t);
    T.t = Vec3(0.0, 0.0, 1.0) + velocity * t;
    return T;
  }
};

/** The scene as seen from the segment starting at `tBegin`, expressed in that
 * knot's body frame with each point's own normalized time.
 */
std::vector<SegmentPoint> observe(
  const std::vector<Vec3> & scene, const Truth & truth, double tBegin, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const CtSegment seg(truth.at(tBegin), truth.at(tBegin + kSegmentInterval));

  std::vector<SegmentPoint> out;
  out.reserve(scene.size() / 4);
  for (const auto & worldPoint : scene) {
    const double alpha = u(rng);
    const SE3 sensor = seg.poseAt(alpha);
    const Vec3 inBody = sensor.inverse() * worldPoint;

    const double range = inBody.norm();
    if (range < 1.0 || range > 30.0) {
      continue;
    }
    SegmentPoint sp;
    sp.p = inBody;
    sp.alpha = alpha;
    out.push_back(sp);
  }
  return out;
}
}  // namespace

TEST(CtMapMatcher, DownsamplingIsDeterministicAndReducesCount)
{
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-5.0, 5.0);

  std::vector<SegmentPoint> pts;
  for (int i = 0; i < 5000; i++) {
    SegmentPoint sp;
    sp.p = Vec3(u(rng), u(rng), u(rng));
    sp.alpha = 0.5;
    pts.push_back(sp);
  }

  const auto a = CtMapMatcher::downsample(pts, 0.4);
  const auto b = CtMapMatcher::downsample(pts, 0.4);

  EXPECT_LT(a.size(), pts.size());
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); i++) {
    EXPECT_EQ(a[i].p, b[i].p);
  }
}

TEST(CtMapMatcher, FindsCorrespondencesAgainstAPopulatedMap)
{
  std::mt19937 rng(2);
  const auto scene = makeScene();
  const Truth truth;

  CtMapMatcher matcher;
  const CtSegment seg0(truth.at(0.0), truth.at(kSegmentInterval));
  matcher.insert(seg0, observe(scene, truth, 0.0, rng));

  ASSERT_GT(matcher.pointCount(), 100u);

  const auto pts = CtMapMatcher::downsample(observe(scene, truth, 0.0, rng), 0.4);
  std::vector<PointCorrespondence> corrs;
  matcher.match(seg0, pts, corrs);

  EXPECT_GT(corrs.size(), pts.size() / 4);
  for (const auto & c : corrs) {
    EXPECT_LT(c.localIndex, pts.size());
    EXPECT_TRUE(c.information.allFinite());
    // GICP weights must be positive semidefinite to be a valid metric:
    const Eigen::SelfAdjointEigenSolver<Mat3> es(c.information);
    EXPECT_GT(es.eigenvalues().minCoeff(), -1e-6);
  }
}

TEST(CtMapMatcher, TheEstimatorRecoversAPerturbedSegmentThroughRealMatching)
{
  std::mt19937 rng(3);
  const auto scene = makeScene();
  const Truth truth;

  CtMapMatcher matcher;
  matcher.params.sourceVoxelSize = 0.3;
  matcher.params.mapVoxelSize = 0.3;
  matcher.params.matchThreshold = 1.0f;

  // Build up a map from three segments observed at the true poses:
  for (int k = 0; k < 3; k++) {
    const double t = k * kSegmentInterval;
    const CtSegment seg(truth.at(t), truth.at(t + kSegmentInterval));
    matcher.insert(seg, observe(scene, truth, t, rng));
  }
  ASSERT_GT(matcher.pointCount(), 1000u);

  // A fourth segment, whose knots start away from the truth:
  const double t3 = 3 * kSegmentInterval;
  std::vector<Knot> knots(2);
  knots[0].t = t3;
  knots[0].state.T = truth.at(t3);
  knots[1].t = t3 + kSegmentInterval;
  knots[1].state.T = truth.at(t3 + kSegmentInterval);

  std::vector<Segment> segments(1);
  segments[0].points =
    CtMapMatcher::downsample(observe(scene, truth, t3, rng), matcher.params.sourceVoxelSize);
  ASSERT_GT(segments[0].points.size(), 200u);

  Vec15 off = Vec15::Zero();
  off.head<3>() = Vec3(0.20, -0.15, 0.08);
  off.segment<3>(kIdxRotation) = Vec3(0.01, -0.02, 0.03);
  incKnot(knots[0].state, off);
  incKnot(knots[1].state, off);

  const double errorBefore = std::max(
    (knots[0].state.T.t - truth.at(t3).t).norm(),
    (knots[1].state.T.t - truth.at(t3 + kSegmentInterval).t).norm());
  ASSERT_GT(errorBefore, 0.15);

  WindowOptimizer::Params p;
  p.useImu = false;
  p.twistContinuityWeight = 0.0;
  p.kernel = RobustKernel::Cauchy;
  p.kernelScale = 0.5;
  p.maxIterations = 20;
  p.rematchEvery = 1;
  p.convergenceThreshold = 1e-5;

  WindowOptimizer opt(p);
  const auto r = opt.optimize(
    knots, segments,
    [&matcher](
      std::size_t, const CtSegment & seg, const std::vector<SegmentPoint> & points,
      std::vector<PointCorrespondence> & out) { matcher.match(seg, points, out); },
    MarginalizationPrior{});

  EXPECT_GT(r.inliers, 100u);

  const double errorAfter = std::max(
    (knots[0].state.T.t - truth.at(t3).t).norm(),
    (knots[1].state.T.t - truth.at(t3 + kSegmentInterval).t).norm());

  EXPECT_LT(errorAfter, 0.05) << "before " << errorBefore << " after " << errorAfter;
}
