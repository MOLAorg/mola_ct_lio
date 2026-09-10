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
 * @file   test_imu_factor.cpp
 * @brief  The preintegrated IMU factor: algebra, Jacobians and convergence.
 */
#include <gtest/gtest.h>
#include <mola_ct_lio/ImuFactor.h>

#include <random>

namespace
{
using namespace mola::ct;  // NOLINT(build/namespaces)

const Vec3 kGravity{0.0, 0.0, -9.81};

PreintegratedImu makePim(std::mt19937 & rng, double dt)
{
  std::normal_distribution<double> g(0.0, 1.0);

  PreintegratedImu pim;
  pim.dt = dt;
  pim.dR = so3Exp(Vec3(0.02, -0.03, 0.05));
  pim.dV = Vec3(0.30, -0.10, 0.05);
  pim.dP = Vec3(0.012, -0.004, 0.002);

  // The bias Jacobians only need to be plausible for a derivative check:
  const auto smallRandom = [&](double s) {
    Mat3 m;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        m(i, j) = s * g(rng);
      }
    }
    return m;
  };
  pim.dR_dbg = smallRandom(dt);
  pim.dV_dba = smallRandom(dt);
  pim.dV_dbg = smallRandom(0.1 * dt);
  pim.dP_dba = smallRandom(0.5 * dt * dt);
  pim.dP_dbg = smallRandom(0.05 * dt * dt);

  pim.cov = Mat9::Identity();
  pim.cov.block<3, 3>(0, 0) *= 1e-6;
  pim.cov.block<3, 3>(3, 3) *= 1e-4;
  pim.cov.block<3, 3>(6, 6) *= 1e-5;

  pim.biasAcc = Vec3(0.01, -0.02, 0.03);
  pim.biasGyro = Vec3(-0.001, 0.002, 0.0005);
  return pim;
}

/** The knot j that exactly satisfies the preintegration relations, given knot i. */
KnotState consistentSecondKnot(const PreintegratedImu & pim, const KnotState & si)
{
  const double dt = pim.dt;

  KnotState sj;
  sj.T.R = si.T.R * pim.dR;
  sj.v = si.v + kGravity * dt + si.T.R * pim.dV;
  sj.T.t = si.T.t + si.v * dt + 0.5 * kGravity * dt * dt + si.T.R * pim.dP;
  sj.biasAcc = si.biasAcc;
  sj.biasGyro = si.biasGyro;
  return sj;
}
}  // namespace

TEST(ImuFactor, ResidualVanishesOnAConsistentPair)
{
  std::mt19937 rng(1);
  const PreintegratedImu pim = makePim(rng, 0.04);

  KnotState si;
  si.T.R = so3Exp(Vec3(0.3, -0.2, 1.1));
  si.T.t = Vec3(5.0, -3.0, 1.0);
  si.v = Vec3(2.0, 0.5, -0.1);
  si.biasAcc = pim.biasAcc;
  si.biasGyro = pim.biasGyro;

  const KnotState sj = consistentSecondKnot(pim, si);

  const Vec9 r = imuResidual(pim, si, sj, kGravity);
  EXPECT_LT(r.cwiseAbs().maxCoeff(), 1e-12) << r.transpose();
}

TEST(ImuFactor, JacobianMatchesFiniteDifferences)
{
  std::mt19937 rng(2);
  std::normal_distribution<double> g(0.0, 1.0);

  for (int trial = 0; trial < 100; trial++) {
    const PreintegratedImu pim = makePim(rng, 0.04);

    KnotState si;
    si.T.R = so3Exp(Vec3(g(rng), g(rng), g(rng)));
    si.T.t = 5.0 * Vec3(g(rng), g(rng), g(rng));
    si.v = Vec3(g(rng), g(rng), g(rng));
    si.biasAcc = pim.biasAcc + 0.02 * Vec3(g(rng), g(rng), g(rng));
    si.biasGyro = pim.biasGyro + 0.002 * Vec3(g(rng), g(rng), g(rng));

    // Away from the consistent pair, so no Jacobian block is trivially zero:
    KnotState sj = consistentSecondKnot(pim, si);
    Vec15 off = Vec15::Zero();
    for (int i = 0; i < 15; i++) {
      off[i] = 0.05 * g(rng);
    }
    incKnot(sj, off);

    Mat9x30 analytic;
    const Vec9 unusedResidual = imuResidualAndJacobian(pim, si, sj, kGravity, analytic);
    (void)unusedResidual;

    constexpr double kEps = 1e-7;
    Mat9x30 numeric;
    for (int i = 0; i < 30; i++) {
      Vec15 di = Vec15::Zero();
      Vec15 dj = Vec15::Zero();
      (i < 15 ? di : dj)[i % 15] = kEps;

      KnotState siPlus = si;
      KnotState sjPlus = sj;
      incKnot(siPlus, di);
      incKnot(sjPlus, dj);

      (i < 15 ? di : dj)[i % 15] = -kEps;
      KnotState siMinus = si;
      KnotState sjMinus = sj;
      incKnot(siMinus, di);
      incKnot(sjMinus, dj);

      numeric.col(i) = (imuResidual(pim, siPlus, sjPlus, kGravity) -
                        imuResidual(pim, siMinus, sjMinus, kGravity)) /
                       (2.0 * kEps);
    }

    const double err = (analytic - numeric).cwiseAbs().maxCoeff();
    EXPECT_LT(err, 1e-4) << "trial " << trial << "\nanalytic:\n"
                         << analytic << "\nnumeric:\n"
                         << numeric;
  }
}

TEST(ImuFactor, GaussNewtonRecoversTheSecondKnot)
{
  std::mt19937 rng(3);
  const PreintegratedImu pim = makePim(rng, 0.04);

  KnotState si;
  si.T.R = so3Exp(Vec3(0.1, 0.2, -0.4));
  si.T.t = Vec3(-1.0, 2.0, 0.3);
  si.v = Vec3(1.5, -0.2, 0.05);
  si.biasAcc = pim.biasAcc;
  si.biasGyro = pim.biasGyro;

  const KnotState truth = consistentSecondKnot(pim, si);

  KnotState sj = truth;
  Vec15 off = Vec15::Zero();
  off.head<3>() = Vec3(0.05, -0.04, 0.02);
  off.segment<3>(kIdxRotation) = Vec3(0.01, 0.02, -0.015);
  off.segment<3>(kIdxVelocity) = Vec3(0.2, -0.1, 0.05);
  incKnot(sj, off);

  for (int it = 0; it < 20; it++) {
    const ImuBlock blk = assembleImuBlock(pim, si, sj, kGravity);

    // Knot i is held fixed here, so only the second half of the system is solved:
    const Eigen::Matrix<double, 15, 15> H =
      blk.H.bottomRightCorner<15, 15>() + 1e-9 * Eigen::Matrix<double, 15, 15>::Identity();
    const Vec15 step = H.ldlt().solve(blk.g.tail<15>());
    incKnot(sj, step);
  }

  EXPECT_LT((sj.T.t - truth.T.t).norm(), 1e-6);
  EXPECT_LT((sj.v - truth.v).norm(), 1e-6);
  EXPECT_LT(so3Log(truth.T.R.transpose() * sj.T.R).norm(), 1e-6);
}

TEST(ImuFactor, BiasRandomWalkPullsBiasesTogether)
{
  KnotState si;
  KnotState sj;
  si.biasAcc = Vec3(0.01, 0.0, 0.0);
  sj.biasAcc = Vec3(0.05, 0.0, 0.0);

  const ImuBlock blk = assembleBiasRandomWalkBlock(si, sj, 0.04, 1e-3, 1e-4);

  EXPECT_LT((blk.H - blk.H.transpose()).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_GT(blk.chi2, 0.0);

  // The gradient must push knot i up and knot j down, i.e. towards each other.
  EXPECT_GT(blk.g[kIdxBiasAcc], 0.0);
  EXPECT_LT(blk.g[kKnotDim + kIdxBiasAcc], 0.0);
}
