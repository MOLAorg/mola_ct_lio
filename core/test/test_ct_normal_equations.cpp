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
 * @file   test_ct_normal_equations.cpp
 * @brief  The segment assembly, checked as a gradient and end to end.
 *
 * Sign conventions between the residual, the Jacobian and the increment are
 * the classic place for a silent error: the wrong one still converges, just
 * to somewhere else. The gradient test pins them, and the recovery test
 * exercises the whole chain including the update step.
 */
#include <gtest/gtest.h>
#include <mola_ct_lio/CtNormalEquations.h>

#include <random>

namespace
{
using namespace mola::ct;  // NOLINT(build/namespaces)

struct Scene
{
  SE3 Tb;
  SE3 Te;
  std::vector<SegmentPoint> points;
  std::vector<PointCorrespondence> correspondences;
};

/** Builds a segment whose correspondences are exact for the given knots, so
 * that the true minimum of the cost is known and its residual is zero.
 */
Scene makeScene(std::mt19937 & rng, std::size_t n, bool planeInformation)
{
  std::normal_distribution<double> g(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.0, 1.0);

  Scene s;
  s.Tb.t = Vec3(1.0, -2.0, 0.5);
  s.Tb.R = so3Exp(Vec3(0.05, -0.03, 0.9));
  s.Te = s.Tb * SE3(so3Exp(Vec3(0.01, 0.02, 0.06)), Vec3(0.4, 0.05, -0.02));

  const CtSegment truth(s.Tb, s.Te);

  s.points.reserve(n);
  s.correspondences.reserve(n);
  for (std::size_t i = 0; i < n; i++) {
    SegmentPoint sp;
    sp.p = 15.0 * Vec3(g(rng), g(rng), g(rng));
    sp.alpha = u(rng);
    s.points.push_back(sp);

    PointCorrespondence c;
    c.localIndex = static_cast<uint32_t>(i);
    c.globalPoint = truth.poseAt(sp.alpha) * sp.p;
    if (planeInformation) {
      // Rank-one weighting, i.e. the point-to-plane degenerate case of GICP:
      const Vec3 n = Vec3(g(rng), g(rng), g(rng)).normalized();
      c.information = n * n.transpose();
    } else {
      c.information = Mat3::Identity();
    }
    s.correspondences.push_back(c);
  }
  return s;
}

/** The cost the assembly is meant to be the Gauss-Newton model of. */
double cost(const Scene & s, const SE3 & Tb, const SE3 & Te)
{
  const CtSegment seg(Tb, Te);
  double total = 0;
  for (const auto & c : s.correspondences) {
    const SegmentPoint & sp = s.points[c.localIndex];
    const Vec3 r = c.globalPoint - (seg.poseAt(sp.alpha) * sp.p);
    total += r.dot(c.information * r);
  }
  return total;
}
}  // namespace

TEST(CtNormalEquations, GradientMatchesTheCost)
{
  std::mt19937 rng(3);
  const Scene s = makeScene(rng, 60, false);

  // Away from the minimum, so the gradient is not trivially zero:
  SE3 Tb = s.Tb;
  SE3 Te = s.Te;
  Vec6 off;
  off << 0.10, -0.05, 0.03, 0.02, -0.01, 0.04;
  incPose(Tb, off);
  incPose(Te, -off);

  const CtSegment seg(Tb, Te);
  const LidarBlock blk =
    assembleSegmentBlock(seg, seg, s.points, s.correspondences, RobustKernel::None, 1.0);

  constexpr double kEps = 1e-6;
  for (int i = 0; i < 12; i++) {
    Vec12 d = Vec12::Zero();

    d[i] = kEps;
    SE3 bp = Tb;
    SE3 ep = Te;
    incPose(bp, d.head<6>());
    incPose(ep, d.tail<6>());

    d[i] = -kEps;
    SE3 bm = Tb;
    SE3 em = Te;
    incPose(bm, d.head<6>());
    incPose(em, d.tail<6>());

    const double numericGrad = (cost(s, bp, ep) - cost(s, bm, em)) / (2.0 * kEps);

    // C(d) ~= |r - J d|^2_Omega, so dC/dd at zero is -2 * g.
    EXPECT_NEAR(numericGrad, -2.0 * blk.g[i], 1e-3 * std::max(1.0, std::abs(blk.g[i])))
      << "component " << i;
  }
}

TEST(CtNormalEquations, GaussNewtonRecoversTheKnots)
{
  for (const bool planeInformation : {false, true}) {
    std::mt19937 rng(planeInformation ? 5 : 4);
    const Scene s = makeScene(rng, 400, planeInformation);

    SE3 Tb = s.Tb;
    SE3 Te = s.Te;
    Vec6 off;
    off << 0.25, -0.18, 0.09, 0.03, 0.02, -0.04;
    incPose(Tb, off);
    incPose(Te, 0.5 * off);

    for (int it = 0; it < 25; it++) {
      const CtSegment seg(Tb, Te);
      const LidarBlock blk =
        assembleSegmentBlock(seg, seg, s.points, s.correspondences, RobustKernel::None, 1.0);

      // A small Levenberg damping keeps the rank-one case invertible:
      const Mat12 H = blk.H + 1e-9 * Mat12::Identity();
      const Vec12 step = H.ldlt().solve(blk.g);

      incPose(Tb, step.head<6>());
      incPose(Te, step.tail<6>());
    }

    EXPECT_LT((Tb.t - s.Tb.t).norm(), 1e-6) << "plane=" << planeInformation;
    EXPECT_LT((Te.t - s.Te.t).norm(), 1e-6) << "plane=" << planeInformation;
    EXPECT_LT(so3Log(s.Tb.R.transpose() * Tb.R).norm(), 1e-6) << "plane=" << planeInformation;
    EXPECT_LT(so3Log(s.Te.R.transpose() * Te.R).norm(), 1e-6) << "plane=" << planeInformation;
  }
}

/** The core sum is what the balance reads as the sensor's noise scale, so it
 * has to be a strict subset selected by residual size, and it has to fall back
 * to the whole population when no radius is asked for. Both halves matter: the
 * first is the property that makes the estimate independent of the acceptance
 * radius, the second is what keeps the default behaviour untouched.
 */
TEST(CtNormalEquations, CoreChiSquareSelectsByResidualAndDefaultsToEverything)
{
  std::mt19937 rng(17);
  const Scene s = makeScene(rng, 500, true);

  // The scene's correspondences are exact, so the residuals only exist away
  // from the truth. Displace the segment to get a spread of them.
  const SE3 offset(so3Exp(Vec3(0.004, -0.002, 0.006)), Vec3(0.05, -0.03, 0.02));
  const CtSegment seg(s.Tb * offset, s.Te * offset);

  const LidarBlock all =
    assembleSegmentBlock(seg, seg, s.points, s.correspondences, RobustKernel::Cauchy, 0.5);
  ASSERT_GT(all.chi2, 0.0) << "the displaced segment must produce residuals";

  // No radius, and a radius past every residual, both mean the whole set.
  EXPECT_EQ(all.coreInliers, all.inliers);
  EXPECT_NEAR(all.coreChi2, all.chi2, 1e-12);

  const LidarBlock wide =
    assembleSegmentBlock(seg, seg, s.points, s.correspondences, RobustKernel::Cauchy, 0.5, 1e6);
  EXPECT_EQ(wide.coreInliers, wide.inliers);
  EXPECT_NEAR(wide.coreChi2, wide.chi2, 1e-12);

  const LidarBlock tight =
    assembleSegmentBlock(seg, seg, s.points, s.correspondences, RobustKernel::Cauchy, 0.5, 1e-9);
  EXPECT_EQ(tight.coreInliers, 0u);
  EXPECT_NEAR(tight.coreChi2, 0.0, 1e-12);

  // Narrowing the radius may only remove correspondences, never add them, and
  // the system it assembles must not move with it: the radius selects what the
  // balance reads, not what the solver fits.
  double previous = static_cast<double>(all.inliers) + 1.0;
  bool sawPartialSelection = false;
  for (const double radius : {10.0, 5.0, 2.0, 1.0, 0.5}) {
    const LidarBlock b = assembleSegmentBlock(
      seg, seg, s.points, s.correspondences, RobustKernel::Cauchy, 0.5, radius);
    EXPECT_LE(static_cast<double>(b.coreInliers), previous) << "radius=" << radius;
    previous = static_cast<double>(b.coreInliers);
    sawPartialSelection = sawPartialSelection || (b.coreInliers > 0 && b.coreInliers < b.inliers);

    EXPECT_LE(b.coreChi2, b.chi2 + 1e-12) << "radius=" << radius;
    EXPECT_EQ(b.inliers, all.inliers) << "radius=" << radius;
    EXPECT_LT((b.H - all.H).cwiseAbs().maxCoeff(), 1e-12) << "radius=" << radius;
    EXPECT_LT((b.g - all.g).cwiseAbs().maxCoeff(), 1e-12) << "radius=" << radius;
    EXPECT_NEAR(b.chi2, all.chi2, 1e-12) << "radius=" << radius;
  }
  EXPECT_TRUE(sawPartialSelection) << "no radius actually split the population";
}

TEST(CtNormalEquations, HessianIsSymmetricAndPositiveSemidefinite)
{
  std::mt19937 rng(9);
  const Scene s = makeScene(rng, 200, true);

  const CtSegment seg(s.Tb, s.Te);
  const LidarBlock blk =
    assembleSegmentBlock(seg, seg, s.points, s.correspondences, RobustKernel::Cauchy, 0.5);

  EXPECT_LT((blk.H - blk.H.transpose()).cwiseAbs().maxCoeff(), 1e-9);

  const Eigen::SelfAdjointEigenSolver<Mat12> es(blk.H);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-9);
}

#if defined(MOLA_CT_LIO_HAS_TBB)
#include <tbb/global_control.h>

/** The assembly is parallel, so its floating-point summation order must not
 * depend on how many threads happen to run it. That is the whole reason for
 * using the deterministic reduction with a fixed grain, and it is the property
 * the estimator's bit-exactness rests on, so it is asserted rather than
 * assumed.
 */
TEST(CtNormalEquations, AssemblyIsIndependentOfTheThreadCount)
{
  std::mt19937 rng(31);
  const Scene s = makeScene(rng, 20000, true);

  const CtSegment seg(s.Tb, s.Te);

  const auto assembleWith = [&](std::size_t threads) {
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, threads);
    return assembleSegmentBlock(seg, seg, s.points, s.correspondences, RobustKernel::Cauchy, 0.5);
  };

  const LidarBlock one = assembleWith(1);
  const LidarBlock four = assembleWith(4);
  const LidarBlock many = assembleWith(32);

  EXPECT_EQ(one.inliers, four.inliers);
  EXPECT_EQ(one.inliers, many.inliers);
  EXPECT_EQ(one.chi2, four.chi2);
  EXPECT_EQ(one.chi2, many.chi2);
  EXPECT_EQ(one.H, four.H);
  EXPECT_EQ(one.H, many.H);
  EXPECT_EQ(one.g, four.g);
  EXPECT_EQ(one.g, many.g);
}
#endif
