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
 * @file   test_window_optimizer.cpp
 * @brief  The whole estimator core, on a synthetic trajectory and map.
 *
 * A constant-velocity ground truth is exactly representable by the segment
 * interpolation, so any residual error here belongs to the estimator and not
 * to the parameterization. The matcher is perfect by construction, which
 * isolates the optimization from the association.
 */
#include <gtest/gtest.h>
#include <mola_ct_lio/WindowOptimizer.h>

#include <random>

namespace
{
using namespace mola::ct;  // NOLINT(build/namespaces)

const Vec3 kGravity{0.0, 0.0, -9.81};
constexpr double kSegmentInterval = 0.04;

/** A ground truth with constant linear and angular velocity, which the segment
 * interpolation reproduces exactly.
 */
struct Truth
{
  SE3 T0;
  Vec3 velocity{2.0, 0.3, -0.05};
  Vec3 angularVelocity{0.02, -0.01, 0.25};

  [[nodiscard]] KnotState at(double t) const
  {
    KnotState s;
    s.T.R = T0.R * so3Exp(angularVelocity * t);
    s.T.t = T0.t + velocity * t;
    s.v = velocity;
    return s;
  }
};

/** The preintegrated measurement that exactly matches a pair of true states. */
PreintegratedImu consistentPim(const KnotState & si, const KnotState & sj, double dt)
{
  PreintegratedImu pim;
  pim.dt = dt;

  const Mat3 R_i_t = si.T.R.transpose();
  pim.dR = R_i_t * sj.T.R;
  pim.dV = R_i_t * (sj.v - si.v - kGravity * dt);
  pim.dP = R_i_t * (sj.T.t - si.T.t - si.v * dt - 0.5 * kGravity * dt * dt);

  pim.cov = Mat9::Identity();
  pim.cov.block<3, 3>(0, 0) *= 1e-6;
  pim.cov.block<3, 3>(3, 3) *= 1e-4;
  pim.cov.block<3, 3>(6, 6) *= 1e-6;
  return pim;
}

struct World
{
  Truth truth;
  std::vector<Knot> knots;
  std::vector<Segment> segments;
  /// The map point each source point came from, per segment.
  std::vector<std::vector<Vec3>> mapPoints;
};

/** Builds a window of `knotCount` knots with `pointsPerSegment` exact
 * observations of a random scene in each segment.
 */
World makeWorld(std::mt19937 & rng, int knotCount, int pointsPerSegment, bool withImu)
{
  std::normal_distribution<double> g(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.0, 1.0);

  World w;
  w.truth.T0.R = so3Exp(Vec3(0.02, -0.01, 0.4));
  w.truth.T0.t = Vec3(3.0, -1.0, 0.5);

  for (int k = 0; k < knotCount; k++) {
    Knot knot;
    knot.t = k * kSegmentInterval;
    knot.state = w.truth.at(knot.t);
    w.knots.push_back(knot);
  }

  for (int k = 0; k + 1 < knotCount; k++) {
    Segment seg;
    std::vector<Vec3> segmentMapPoints;

    const CtSegment ct(w.knots[k].state.T, w.knots[k + 1].state.T);

    for (int i = 0; i < pointsPerSegment; i++) {
      // A map point somewhere around the sensor:
      const Vec3 mapPoint = w.truth.at(w.knots[k].t).T.t + 20.0 * Vec3(g(rng), g(rng), g(rng));

      SegmentPoint sp;
      sp.alpha = u(rng);
      // Where that map point was seen from, at the point's own time:
      sp.p = ct.poseAt(sp.alpha).inverse() * mapPoint;

      seg.points.push_back(sp);
      segmentMapPoints.push_back(mapPoint);
    }

    if (withImu) {
      seg.imu = consistentPim(w.knots[k].state, w.knots[k + 1].state, kSegmentInterval);
      seg.hasImu = true;
    }

    w.segments.push_back(std::move(seg));
    w.mapPoints.push_back(std::move(segmentMapPoints));
  }

  return w;
}

/** A matcher that always returns the true correspondence, so the test isolates
 * the optimization from the association.
 */
MatchFunction perfectMatcher(const World & w)
{
  return [&w](
           std::size_t segmentIndex, const CtSegment &, const std::vector<SegmentPoint> & points,
           std::vector<PointCorrespondence> & out) {
    const auto & mapPoints = w.mapPoints[segmentIndex];
    for (std::size_t i = 0; i < points.size(); i++) {
      PointCorrespondence c;
      c.localIndex = static_cast<uint32_t>(i);
      c.globalPoint = mapPoints[i];
      c.information = Mat3::Identity();
      out.push_back(c);
    }
  };
}

double worstKnotError(const std::vector<Knot> & knots, const Truth & truth)
{
  double worst = 0;
  for (const auto & k : knots) {
    const KnotState want = truth.at(k.t);
    worst = std::max(worst, (k.state.T.t - want.T.t).norm());
    worst = std::max(worst, so3Log(want.T.R.transpose() * k.state.T.R).norm());
  }
  return worst;
}

void perturb(std::vector<Knot> & knots, double scale, std::mt19937 & rng)
{
  std::normal_distribution<double> g(0.0, 1.0);
  for (auto & k : knots) {
    Vec15 inc = Vec15::Zero();
    for (int i = 0; i < 6; i++) {
      inc[i] = scale * g(rng) * (i < 3 ? 1.0 : 0.1);
    }
    incKnot(k.state, inc);
  }
}
}  // namespace

TEST(WindowOptimizer, RecoversTheTrajectoryWithoutImu)
{
  std::mt19937 rng(21);
  World w = makeWorld(rng, 4, 300, false);

  perturb(w.knots, 0.15, rng);
  ASSERT_GT(worstKnotError(w.knots, w.truth), 0.05);

  WindowOptimizer::Params p;
  p.useImu = false;
  p.kernel = RobustKernel::None;
  p.convergenceThreshold = 1e-9;
  p.maxIterations = 30;
  // The twist-continuity term biases a window that is already exactly
  // determined by the LiDAR, so it is off for this check.
  p.twistContinuityWeight = 0.0;

  WindowOptimizer opt(p);
  const auto r = opt.optimize(w.knots, w.segments, perfectMatcher(w), MarginalizationPrior{});

  EXPECT_GT(r.inliers, 0u);
  EXPECT_LT(worstKnotError(w.knots, w.truth), 1e-5);
}

TEST(WindowOptimizer, RecoversTheTrajectoryWithImu)
{
  std::mt19937 rng(22);
  World w = makeWorld(rng, 4, 300, true);

  perturb(w.knots, 0.15, rng);

  WindowOptimizer::Params p;
  p.useImu = true;
  p.gravity = kGravity;
  p.kernel = RobustKernel::None;
  p.convergenceThreshold = 1e-9;
  p.maxIterations = 30;

  WindowOptimizer opt(p);
  const auto r = opt.optimize(w.knots, w.segments, perfectMatcher(w), MarginalizationPrior{});

  EXPECT_GT(r.inliers, 0u);
  EXPECT_LT(worstKnotError(w.knots, w.truth), 1e-4);

  // The IMU factors are consistent with the truth, so the velocities must have
  // been pulled onto it too, even though no LiDAR residual sees them.
  for (const auto & k : w.knots) {
    EXPECT_LT((k.state.v - w.truth.at(k.t).v).norm(), 1e-3);
  }
}

TEST(WindowOptimizer, TheTwistContinuityTermIsHarmlessOnAConstantVelocityTruth)
{
  std::mt19937 rng(23);
  World w = makeWorld(rng, 4, 300, false);
  perturb(w.knots, 0.10, rng);

  WindowOptimizer::Params p;
  p.useImu = false;
  p.kernel = RobustKernel::None;
  p.convergenceThreshold = 1e-9;
  p.maxIterations = 30;
  p.twistContinuityWeight = 2.0;

  WindowOptimizer opt(p);
  opt.optimize(w.knots, w.segments, perfectMatcher(w), MarginalizationPrior{});

  // A constant-velocity trajectory has a constant twist, so the term's own
  // residual is zero there and it must not pull the solution away.
  EXPECT_LT(worstKnotError(w.knots, w.truth), 1e-4);
}

TEST(WindowOptimizer, MarginalizingAndSlidingKeepsTheTrajectory)
{
  std::mt19937 rng(24);
  World w = makeWorld(rng, 5, 300, true);

  WindowOptimizer::Params p;
  p.useImu = true;
  p.gravity = kGravity;
  p.kernel = RobustKernel::None;
  p.convergenceThreshold = 1e-10;
  p.maxIterations = 30;

  WindowOptimizer opt(p);

  // First window: knots 0..3.
  std::vector<Knot> window(w.knots.begin(), w.knots.begin() + 4);
  std::vector<Segment> windowSegments(w.segments.begin(), w.segments.begin() + 3);
  perturb(window, 0.12, rng);

  opt.optimize(window, windowSegments, perfectMatcher(w), MarginalizationPrior{});
  ASSERT_LT(worstKnotError(window, w.truth), 1e-3);

  // Drop the oldest knot, keeping what it had to say about the rest.
  const MarginalizationPrior prior = marginalizeLeadingKnots(opt.lastSystem(), 1);
  ASSERT_TRUE(prior.valid);

  std::vector<Knot> slid(window.begin() + 1, window.end());
  for (auto & k : slid) {
    k.linearized = true;
    k.linearizationPoint = k.state;
    k.priorAnchor = k.state;
  }
  slid.push_back(w.knots[4]);

  std::vector<Segment> slidSegments(w.segments.begin() + 1, w.segments.begin() + 4);

  // The matcher indexes by the segment's position in the window, which has
  // shifted by one now.
  const World & world = w;
  MatchFunction shifted = [&world](
                            std::size_t segmentIndex, const CtSegment &,
                            const std::vector<SegmentPoint> & points,
                            std::vector<PointCorrespondence> & out) {
    const auto & mapPoints = world.mapPoints[segmentIndex + 1];
    for (std::size_t i = 0; i < points.size(); i++) {
      PointCorrespondence c;
      c.localIndex = static_cast<uint32_t>(i);
      c.globalPoint = mapPoints[i];
      c.information = Mat3::Identity();
      out.push_back(c);
    }
  };

  perturb(slid, 0.05, rng);
  opt.optimize(slid, slidSegments, shifted, prior);

  EXPECT_LT(worstKnotError(slid, w.truth), 1e-3);
}

TEST(WindowOptimizer, IsBitwiseRepeatable)
{
  std::mt19937 rng(25);
  const World reference = makeWorld(rng, 4, 500, true);

  WindowOptimizer::Params p;
  p.useImu = true;
  p.gravity = kGravity;
  p.maxIterations = 12;

  const auto run = [&]() {
    World w = reference;
    std::mt19937 perturbRng(99);
    perturb(w.knots, 0.1, perturbRng);
    WindowOptimizer opt(p);
    opt.optimize(w.knots, w.segments, perfectMatcher(w), MarginalizationPrior{});
    return w.knots;
  };

  const auto a = run();
  const auto b = run();

  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); i++) {
    EXPECT_EQ(a[i].state.T.t, b[i].state.T.t);
    EXPECT_EQ(a[i].state.T.R, b[i].state.T.R);
    EXPECT_EQ(a[i].state.v, b[i].state.v);
  }
}
