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
 * @file   ImuFactor.cpp
 * @brief  Preintegrated IMU factor between two consecutive control knots
 */
#include <mola_ct_lio/ImuFactor.h>

namespace mola::ct
{
namespace
{
/** The bias-corrected deltas, to first order about the linearization point. */
struct CorrectedDelta
{
  Mat3 dR = Mat3::Identity();
  Vec3 dV = Vec3::Zero();
  Vec3 dP = Vec3::Zero();
  Vec3 dBiasGyro = Vec3::Zero();
};

CorrectedDelta correctForBias(const PreintegratedImu & pim, const KnotState & si)
{
  CorrectedDelta out;
  const Vec3 dba = si.biasAcc - pim.biasAcc;
  out.dBiasGyro = si.biasGyro - pim.biasGyro;

  out.dR = pim.dR * so3Exp(pim.dR_dbg * out.dBiasGyro);
  out.dV = pim.dV + pim.dV_dba * dba + pim.dV_dbg * out.dBiasGyro;
  out.dP = pim.dP + pim.dP_dba * dba + pim.dP_dbg * out.dBiasGyro;
  return out;
}

/** Symmetric square root of the information, used to whiten a residual block. */
Mat9 informationOf(const Mat9 & cov)
{
  // A tiny floor keeps a degenerate covariance (a factor spanning almost no
  // time, or a channel the caller left at zero) from producing an infinite
  // weight:
  const Mat9 c = cov + 1e-12 * Mat9::Identity();
  return c.inverse();
}
}  // namespace

Vec9 imuResidual(
  const PreintegratedImu & pim, const KnotState & si, const KnotState & sj, const Vec3 & gravity)
{
  const CorrectedDelta d = correctForBias(pim, si);

  const Mat3 R_i_t = si.T.R.transpose();
  const double dt = pim.dt;

  const Vec3 A = sj.v - si.v - gravity * dt;
  const Vec3 B = sj.T.t - si.T.t - si.v * dt - 0.5 * gravity * dt * dt;

  Vec9 r;
  r.segment<3>(0) = so3Log(d.dR.transpose() * R_i_t * sj.T.R);
  r.segment<3>(3) = R_i_t * A - d.dV;
  r.segment<3>(6) = R_i_t * B - d.dP;
  return r;
}

Vec9 imuResidualAndJacobian(
  const PreintegratedImu & pim, const KnotState & si, const KnotState & sj, const Vec3 & gravity,
  Mat9x30 & J)
{
  const CorrectedDelta d = correctForBias(pim, si);

  const Mat3 R_i_t = si.T.R.transpose();
  const Mat3 R_j_t = sj.T.R.transpose();
  const double dt = pim.dt;

  const Vec3 A = sj.v - si.v - gravity * dt;
  const Vec3 B = sj.T.t - si.T.t - si.v * dt - 0.5 * gravity * dt * dt;

  const Vec3 rotationError = so3Log(d.dR.transpose() * R_i_t * sj.T.R);

  Vec9 r;
  r.segment<3>(0) = rotationError;
  r.segment<3>(3) = R_i_t * A - d.dV;
  r.segment<3>(6) = R_i_t * B - d.dP;

  J.setZero();

  const Mat3 jacobianInvRotation = rightJacobianInvSO3(rotationError);

  constexpr int kJ = kKnotDim;

  // --- rotation residual ---
  J.block<3, 3>(0, kIdxRotation) = -jacobianInvRotation * R_j_t * si.T.R;
  J.block<3, 3>(0, kJ + kIdxRotation) = jacobianInvRotation;
  // Through the first-order bias update of the preintegrated rotation:
  J.block<3, 3>(0, kIdxBiasGyro) = -jacobianInvRotation * so3Exp(-rotationError) *
                                   rightJacobianSO3(pim.dR_dbg * d.dBiasGyro) * pim.dR_dbg;

  // --- velocity residual ---
  J.block<3, 3>(3, kIdxRotation) = hat(R_i_t * A);
  J.block<3, 3>(3, kIdxVelocity) = -R_i_t;
  J.block<3, 3>(3, kJ + kIdxVelocity) = R_i_t;
  J.block<3, 3>(3, kIdxBiasAcc) = -pim.dV_dba;
  J.block<3, 3>(3, kIdxBiasGyro) = -pim.dV_dbg;

  // --- position residual ---
  J.block<3, 3>(6, kIdxRotation) = hat(R_i_t * B);
  J.block<3, 3>(6, kIdxPosition) = -R_i_t;
  J.block<3, 3>(6, kJ + kIdxPosition) = R_i_t;
  J.block<3, 3>(6, kIdxVelocity) = -R_i_t * dt;
  J.block<3, 3>(6, kIdxBiasAcc) = -pim.dP_dba;
  J.block<3, 3>(6, kIdxBiasGyro) = -pim.dP_dbg;

  return r;
}

ImuBlock assembleImuBlock(
  const PreintegratedImu & pim, const KnotState & si, const KnotState & sj, const Vec3 & gravity)
{
  Mat9x30 J;
  const Vec9 r = imuResidualAndJacobian(pim, si, sj, gravity, J);

  const Mat9 information = informationOf(pim.cov);

  ImuBlock out;
  const Eigen::Matrix<double, 30, 9> JtOmega = J.transpose() * information;
  out.H.noalias() = JtOmega * J;
  // The residual is defined as h(x), not z - h(x), so the gradient carries the
  // opposite sign of the LiDAR block's and the solver adds them as they come.
  out.g.noalias() = -JtOmega * r;
  out.chi2 = r.dot(information * r);
  return out;
}

ImuBlock assembleBiasRandomWalkBlock(
  const KnotState & si, const KnotState & sj, double dt, double sigmaAcc, double sigmaGyro)
{
  ImuBlock out;

  const double varAcc = sigmaAcc * sigmaAcc * dt;
  const double varGyro = sigmaGyro * sigmaGyro * dt;

  const Vec3 rAcc = sj.biasAcc - si.biasAcc;
  const Vec3 rGyro = sj.biasGyro - si.biasGyro;

  constexpr int kJ = kKnotDim;

  const auto accumulate = [&](int offset, const Vec3 & residual, double variance) {
    const double w = 1.0 / std::max(variance, 1e-18);

    // The residual is b_j - b_i, so its Jacobian is -I on knot i and +I on j.
    for (int k = 0; k < 3; k++) {
      const int ii = offset + k;
      const int jj = kJ + offset + k;

      out.H(ii, ii) += w;
      out.H(jj, jj) += w;
      out.H(ii, jj) -= w;
      out.H(jj, ii) -= w;

      out.g[ii] += w * residual[k];
      out.g[jj] -= w * residual[k];
    }
    out.chi2 += w * residual.squaredNorm();
  };

  accumulate(kIdxBiasAcc, rAcc, varAcc);
  accumulate(kIdxBiasGyro, rGyro, varGyro);

  return out;
}

}  // namespace mola::ct
