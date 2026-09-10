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
 * @file   test_odometry_engine.cpp
 * @brief  The whole system on a simulated run: scans in, trajectory out.
 *
 * Segment cutting, window sliding, the order of marginalization against
 * emission and map insertion, and the bootstrap are all orchestration rather
 * than mathematics, and that is where this is most likely to be wrong.
 */
#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <random>

#include "CtOdometryEngine.h"

namespace
{
using namespace mola;      // NOLINT(build/namespaces)
using namespace mola::ct;  // NOLINT(build/namespaces)

constexpr double kScanPeriod = 0.1;

/** A corridor: two side walls and a floor.
 *
 * Every surface in it is parallel to the direction of travel, so motion along
 * the corridor axis is **not observable** from the geometry. That is a real
 * degeneracy, not an artifact of the simulation, and it is the case worth
 * having a test for.
 *
 * @param breakAxialDegeneracy Adds pillars and an end wall, i.e. surfaces with
 *        a normal along the travel direction, which is what makes the axial
 *        motion observable at all.
 */
std::vector<Vec3> makeScene(bool breakAxialDegeneracy)
{
  std::vector<Vec3> pts;
  for (double x = -10.0; x <= 60.0; x += 0.15) {
    for (double z = 0.0; z <= 3.0; z += 0.15) {
      pts.emplace_back(x, -4.0, z);
      pts.emplace_back(x, 4.0, z);
    }
    for (double y = -4.0; y <= 4.0; y += 0.15) {
      pts.emplace_back(x, y, 0.0);
    }
  }

  if (breakAxialDegeneracy) {
    for (double x0 = 0.0; x0 <= 60.0; x0 += 5.0) {
      for (double dy = -0.4; dy <= 0.4; dy += 0.15) {
        for (double z = 0.0; z <= 2.5; z += 0.15) {
          for (const double y0 : {-3.0, 3.0}) {
            pts.emplace_back(x0, y0 + dy, z);
            pts.emplace_back(x0 + 0.4, y0 + dy, z);
          }
        }
      }
    }
  }
  return pts;
}

/** A platform starting from rest and accelerating, which is what a real one
 * does and what a cold start can actually cope with: an odometry has no way to
 * know it was already moving at the first scan.
 */
struct Truth
{
  Vec3 acceleration{1.0, 0.0, 0.0};
  Vec3 angularVelocity{0.0, 0.0, 0.03};

  [[nodiscard]] Vec3 velocityAt(double t) const { return acceleration * t; }

  [[nodiscard]] SE3 at(double t) const
  {
    SE3 T;
    T.R = so3Exp(angularVelocity * t);
    T.t = Vec3(0.0, 0.0, 1.0) + 0.5 * acceleration * t * t;
    return T;
  }

  /// Proper acceleration in the body frame, i.e. what an accelerometer reads.
  [[nodiscard]] Vec3 properAccelerationAt(double t, const Vec3 & gravity) const
  {
    return at(t).R.transpose() * (acceleration - gravity);
  }
};

/** One sweep: the scene as the sensor saw it over [tScan, tScan + period). */
std::vector<CtOdometryEngine::TimedPoint> simulateScan(
  const std::vector<Vec3> & scene, const Truth & truth, double tScan, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(0.0, 1.0);

  std::vector<CtOdometryEngine::TimedPoint> out;
  out.reserve(scene.size() / 2);
  for (const auto & worldPoint : scene) {
    const double t = tScan + u(rng) * kScanPeriod;
    const SE3 sensor = truth.at(t);
    const Vec3 inBody = sensor.inverse() * worldPoint;

    const double range = inBody.norm();
    if (range < 1.0 || range > 30.0) {
      continue;
    }
    out.push_back(CtOdometryEngine::TimedPoint{inBody, t});
  }
  return out;
}

struct RunOutcome
{
  std::vector<std::pair<double, SE3>> poses;
  double worstError = 0;
  double finalError = 0;
};

RunOutcome run(int scanCount, bool useImu, unsigned seed, bool breakAxialDegeneracy = true)
{
  const auto scene = makeScene(breakAxialDegeneracy);
  const Truth truth;

  CtOdometryEngine engine;
  engine.params.optimizer.useImu = useImu;
  engine.params.optimizer.twistContinuityWeight = useImu ? 0.0 : 2.0;
  engine.params.optimizer.maxIterations = 12;
  engine.params.optimizer.rematchEvery = 2;
  engine.params.matcher.sourceVoxelSize = 0.4;
  engine.params.matcher.mapVoxelSize = 0.4;
  engine.params.matcher.matchThreshold = 1.0f;
  engine.params.minRange = 1.0;
  engine.params.maxRange = 30.0;

  engine.setInitialState(truth.at(0.0));

  RunOutcome outcome;
  const bool verbose = ::getenv("CTLIO_TEST_VERBOSE") != nullptr;
  engine.onPose = [&](double t, const SE3 & pose, const CtOdometryEngine::Diagnostics & d) {
    outcome.poses.emplace_back(t, pose);
    if (verbose) {
      const SE3 want = truth.at(t);
      printf(
        "t=%.3f err=%.4f  est=(%7.3f %7.3f %7.3f) gt=(%7.3f %7.3f %7.3f) "
        "v=(%6.3f %6.3f %6.3f) it=%2d conv=%d inl=%6zu map=%7zu seg=%5zu chi2=%.3e\n",
        t, (pose.t - want.t).norm(), pose.t.x(), pose.t.y(), pose.t.z(), want.t.x(), want.t.y(),
        want.t.z(), d.velocity.x(), d.velocity.y(), d.velocity.z(), d.iterations,
        d.converged ? 1 : 0, d.inliers, d.mapPoints, d.segmentPoints, d.chi2);
    }
  };

  std::mt19937 rng(seed);
  for (int s = 0; s < scanCount; s++) {
    const double tScan = s * kScanPeriod;
    if (useImu) {
      // A perfect IMU for the true motion.
      for (int i = 0; i < 20; i++) {
        const double t = tScan + i * (kScanPeriod / 20.0);
        engine.addImuSample(
          t, truth.properAccelerationAt(t, engine.params.optimizer.gravity), truth.angularVelocity);
      }
    }
    engine.addPoints(simulateScan(scene, truth, tScan, rng));
  }
  engine.finish();

  for (const auto & [t, pose] : outcome.poses) {
    const double e = (pose.t - truth.at(t).t).norm();
    outcome.worstError = std::max(outcome.worstError, e);
    outcome.finalError = e;
  }
  return outcome;
}
}  // namespace

TEST(CtOdometryEngine, EmitsPosesAtTheSegmentRate)
{
  const auto outcome = run(10, false, 1);

  // Ten scans of 0.1 s cut at 0.04 s give about 25 segments, less the window
  // that has not been filled yet.
  EXPECT_GT(outcome.poses.size(), 15u);
  EXPECT_LT(outcome.poses.size(), 30u);

  for (std::size_t i = 1; i < outcome.poses.size(); i++) {
    EXPECT_NEAR(outcome.poses[i].first - outcome.poses[i - 1].first, 0.04, 1e-9);
  }
}

TEST(CtOdometryEngine, TracksTheTrajectoryLidarOnly)
{
  const auto outcome = run(20, false, 2);
  EXPECT_LT(outcome.finalError, 0.05) << "worst " << outcome.worstError;
}

TEST(CtOdometryEngine, TracksTheTrajectoryWithImu)
{
  const auto outcome = run(20, true, 3);
  EXPECT_LT(outcome.finalError, 0.05) << "worst " << outcome.worstError;
}

/** In a scene whose every surface runs along the direction of travel, the
 * LiDAR cannot see axial motion at all, and a LiDAR-only estimator has nothing
 * to stop it drifting. The IMU is the only thing that observes it, so this is
 * the case that says whether the inertial coupling is actually carrying
 * information rather than just being present.
 */
TEST(CtOdometryEngine, ImuRescuesAnAxiallyDegenerateScene)
{
  const auto lidarOnly = run(20, false, 4, false);
  const auto withImu = run(20, true, 5, false);

  EXPECT_GT(lidarOnly.finalError, 0.10) << "the corridor was expected to be degenerate";
  EXPECT_LT(withImu.finalError, 0.05);
  EXPECT_LT(withImu.finalError * 5.0, lidarOnly.finalError);
}

TEST(CtOdometryEngine, IsBitwiseRepeatable)
{
  const auto a = run(10, true, 7);
  const auto b = run(10, true, 7);

  ASSERT_EQ(a.poses.size(), b.poses.size());
  for (std::size_t i = 0; i < a.poses.size(); i++) {
    EXPECT_EQ(a.poses[i].first, b.poses[i].first);
    EXPECT_EQ(a.poses[i].second.t, b.poses[i].second.t);
    EXPECT_EQ(a.poses[i].second.R, b.poses[i].second.R);
  }
}
