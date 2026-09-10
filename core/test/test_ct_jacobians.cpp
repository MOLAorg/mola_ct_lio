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
 * @file   test_ct_jacobians.cpp
 * @brief  Numerical validation of the continuous-time interpolation Jacobians.
 *
 * These are the mathematical core of the method: an error here does not make
 * anything fail, it just makes the optimizer converge slowly or to the wrong
 * place. They are checked against finite differences of the very same
 * increment convention the solver applies.
 */
#include <gtest/gtest.h>
#include <mola_ct_lio/CtSegment.h>

#include <random>

namespace
{
using namespace mola::ct;  // NOLINT(build/namespaces)

SE3 randomPose(std::mt19937 & rng, double translationScale, double rotationScale)
{
  std::normal_distribution<double> g(0.0, 1.0);
  SE3 T;
  T.t = translationScale * Vec3(g(rng), g(rng), g(rng));
  T.R = so3Exp(rotationScale * Vec3(g(rng), g(rng), g(rng)));
  return T;
}

/** The world point produced by the segment defined by the two knots, for a
 * body-frame point observed at normalized time `alpha`.
 */
Vec3 worldPoint(const SE3 & Tb, const SE3 & Te, double alpha, const Vec3 & pointInBody)
{
  const CtSegment seg(Tb, Te);
  return seg.poseAt(alpha) * pointInBody;
}

/** Finite-difference Jacobian of worldPoint() with respect to the stacked
 * increments of both knots, using the solver's own increment convention.
 */
Mat3x12 numericJacobian(const SE3 & Tb, const SE3 & Te, double alpha, const Vec3 & pointInBody)
{
  constexpr double kEps = 1e-6;

  Mat3x12 J;
  for (int i = 0; i < 12; i++) {
    Vec12 delta = Vec12::Zero();

    delta[i] = kEps;
    SE3 TbPlus = Tb;
    SE3 TePlus = Te;
    incPose(TbPlus, delta.head<6>());
    incPose(TePlus, delta.tail<6>());

    delta[i] = -kEps;
    SE3 TbMinus = Tb;
    SE3 TeMinus = Te;
    incPose(TbMinus, delta.head<6>());
    incPose(TeMinus, delta.tail<6>());

    const Vec3 plus = worldPoint(TbPlus, TePlus, alpha, pointInBody);
    const Vec3 minus = worldPoint(TbMinus, TeMinus, alpha, pointInBody);

    J.col(i) = (plus - minus) / (2.0 * kEps);
  }
  return J;
}
}  // namespace

TEST(CtJacobians, MatchFiniteDifferences)
{
  std::mt19937 rng(42);
  std::normal_distribution<double> g(0.0, 1.0);

  for (int trial = 0; trial < 200; trial++) {
    // A knot pair spans one segment, so the relative motion is small; the
    // larger rotations here are deliberately beyond anything realistic.
    const SE3 Tb = randomPose(rng, 10.0, 1.0);
    const SE3 Te = Tb * randomPose(rng, 0.5, 0.3);

    const Vec3 pointInBody = 20.0 * Vec3(g(rng), g(rng), g(rng));
    const double alpha = std::uniform_real_distribution<double>(0.0, 1.0)(rng);

    const CtSegment seg(Tb, Te);
    const Mat3x12 analytic =
      CtSegment::pointJacobian(seg.poseAt(alpha), pointInBody) * seg.interpolationJacobian(alpha);

    const Mat3x12 numeric = numericJacobian(Tb, Te, alpha, pointInBody);

    const double err = (analytic - numeric).cwiseAbs().maxCoeff();
    EXPECT_LT(err, 1e-4) << "trial " << trial << " alpha " << alpha << "\nanalytic:\n"
                         << analytic << "\nnumeric:\n"
                         << numeric;
  }
}

TEST(CtJacobians, EndpointsReproduceTheKnots)
{
  std::mt19937 rng(7);

  for (int trial = 0; trial < 50; trial++) {
    const SE3 Tb = randomPose(rng, 5.0, 1.0);
    const SE3 Te = Tb * randomPose(rng, 0.5, 0.3);
    const CtSegment seg(Tb, Te);

    const SE3 at0 = seg.poseAt(0.0);
    const SE3 at1 = seg.poseAt(1.0);

    EXPECT_LT((at0.t - Tb.t).norm(), 1e-12);
    EXPECT_LT((at0.R - Tb.R).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((at1.t - Te.t).norm(), 1e-12);
    EXPECT_LT((at1.R - Te.R).cwiseAbs().maxCoeff(), 1e-10);
  }
}

TEST(CtJacobians, SO3RightJacobianInverseIsConsistent)
{
  std::mt19937 rng(11);
  std::normal_distribution<double> g(0.0, 1.0);

  for (int trial = 0; trial < 100; trial++) {
    const double scale = trial < 10 ? 1e-9 : 1.0;
    const Vec3 phi = scale * Vec3(g(rng), g(rng), g(rng));

    const Mat3 product = rightJacobianSO3(phi) * rightJacobianInvSO3(phi);
    EXPECT_LT((product - Mat3::Identity()).cwiseAbs().maxCoeff(), 1e-8)
      << "phi " << phi.transpose();
  }
}
