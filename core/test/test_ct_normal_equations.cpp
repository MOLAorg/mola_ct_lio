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
