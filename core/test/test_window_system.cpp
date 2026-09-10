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
 * @file   test_window_system.cpp
 * @brief  Window assembly and the exactness of the marginalization.
 *
 * Marginalization is the one place where a mistake stays invisible: the
 * estimator keeps running and simply carries the wrong amount of information
 * forward. The defining property is checkable exactly, so it is checked
 * exactly here.
 */
#include <gtest/gtest.h>
#include <mola_ct_lio/WindowSystem.h>

#include <random>

namespace
{
using namespace mola::ct;  // NOLINT(build/namespaces)

/** A random symmetric positive-definite matrix of the given size. */
Eigen::MatrixXd randomSpd(std::mt19937 & rng, int n)
{
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::MatrixXd A(n, n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      A(i, j) = g(rng);
    }
  }
  return A.transpose() * A + n * Eigen::MatrixXd::Identity(n, n);
}
}  // namespace

TEST(WindowSystem, MarginalizationReproducesTheFullSolution)
{
  std::mt19937 rng(17);
  std::normal_distribution<double> gauss(0.0, 1.0);

  constexpr int kKnots = 4;
  constexpr int kDim = kKnotDim;
  constexpr int kN = kKnots * kDim;

  for (int trial = 0; trial < 20; trial++) {
    WindowSystem sys(kKnots, kDim);
    sys.H() = randomSpd(rng, kN);
    for (int i = 0; i < kN; i++) {
      sys.g()[i] = gauss(rng);
    }

    const Eigen::VectorXd full = sys.H().ldlt().solve(sys.g());

    for (int drop = 1; drop <= 2; drop++) {
      const MarginalizationPrior prior = marginalizeLeadingKnots(sys, drop);
      ASSERT_TRUE(prior.valid);

      const int remaining = kN - drop * kDim;
      ASSERT_EQ(prior.H.rows(), remaining);

      const Eigen::VectorXd reduced = prior.H.ldlt().solve(prior.g);

      const double err = (reduced - full.tail(remaining)).cwiseAbs().maxCoeff();
      EXPECT_LT(err, 1e-8) << "trial " << trial << " drop " << drop;
    }
  }
}

TEST(WindowSystem, MarginalizationToleratesARankDeficientBlock)
{
  constexpr int kKnots = 3;
  constexpr int kDim = kKnotDim;

  WindowSystem sys(kKnots, kDim);
  // Only the second and third knots are constrained; the first is untouched,
  // as happens when no segment reached it.
  const int n = sys.size();
  sys.H().bottomRightCorner(n - kDim, n - kDim) =
    Eigen::MatrixXd::Identity(n - kDim, n - kDim) * 4.0;
  sys.g().tail(n - kDim).setOnes();

  const MarginalizationPrior prior = marginalizeLeadingKnots(sys, 1);
  ASSERT_TRUE(prior.valid);
  EXPECT_TRUE(prior.H.allFinite());
  EXPECT_TRUE(prior.g.allFinite());

  const Eigen::VectorXd x = prior.H.ldlt().solve(prior.g);
  EXPECT_NEAR(x[0], 0.25, 1e-12);
}

TEST(WindowSystem, PoseBlocksLandOnThePosePartOfTheRightKnots)
{
  constexpr int kKnots = 3;
  WindowSystem sys(kKnots, kKnotDim);

  Mat12 H = Mat12::Zero();
  Vec12 g = Vec12::Zero();
  H(0, 0) = 1.0;    // begin knot, x translation
  H(6, 6) = 2.0;    // end knot, x translation
  H(11, 11) = 3.0;  // end knot, z rotation
  g[0] = 5.0;
  g[11] = 7.0;

  sys.addPosePairBlock(1, H, g);

  EXPECT_DOUBLE_EQ(sys.H()(kKnotDim + kIdxPosition, kKnotDim + kIdxPosition), 1.0);
  EXPECT_DOUBLE_EQ(sys.H()(2 * kKnotDim + kIdxPosition, 2 * kKnotDim + kIdxPosition), 2.0);
  EXPECT_DOUBLE_EQ(sys.H()(2 * kKnotDim + kIdxRotation + 2, 2 * kKnotDim + kIdxRotation + 2), 3.0);
  EXPECT_DOUBLE_EQ(sys.g()[kKnotDim + kIdxPosition], 5.0);
  EXPECT_DOUBLE_EQ(sys.g()[2 * kKnotDim + kIdxRotation + 2], 7.0);

  // Nothing must have touched the first knot:
  EXPECT_DOUBLE_EQ(sys.H().topLeftCorner(kKnotDim, kKnotDim).cwiseAbs().maxCoeff(), 0.0);
}

TEST(WindowSystem, StatePairBlockCoversVelocityAndBias)
{
  WindowSystem sys(2, kKnotDim);

  Mat30 H = Mat30::Zero();
  Vec30 g = Vec30::Zero();
  H(kIdxVelocity, kIdxVelocity) = 9.0;
  H(kKnotDim + kIdxBiasGyro, kKnotDim + kIdxBiasGyro) = 11.0;
  g[kIdxVelocity] = 13.0;

  sys.addStatePairBlock(0, H, g);

  EXPECT_DOUBLE_EQ(sys.H()(kIdxVelocity, kIdxVelocity), 9.0);
  EXPECT_DOUBLE_EQ(sys.H()(kKnotDim + kIdxBiasGyro, kKnotDim + kIdxBiasGyro), 11.0);
  EXPECT_DOUBLE_EQ(sys.g()[kIdxVelocity], 13.0);
}

TEST(WindowSystem, PoseOnlyWindowKeepsTheSameLayout)
{
  // With no IMU a knot is a bare pose, and the very same pose block must land
  // on the very same variables.
  WindowSystem sys(3, 6);

  Mat12 H = Mat12::Zero();
  Vec12 g = Vec12::Zero();
  H(6, 6) = 2.0;
  g[6] = 4.0;

  sys.addPosePairBlock(0, H, g);

  EXPECT_DOUBLE_EQ(sys.H()(6, 6), 2.0);
  EXPECT_DOUBLE_EQ(sys.g()[6], 4.0);
}
