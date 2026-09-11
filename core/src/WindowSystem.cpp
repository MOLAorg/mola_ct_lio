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
 * @file   WindowSystem.cpp
 * @brief  Dense normal equations over a window of control knots
 */
#include <mola_ct_lio/WindowSystem.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace mola::ct
{
namespace
{
/** Scatters a small dense block onto the given variable offsets.
 *
 * `offsets` lists, for each row of the block, where it lands in the system,
 * which keeps a single routine usable for every factor shape here.
 */
template <typename BlockH, typename BlockG>
void scatter(
  Eigen::MatrixXd & H, Eigen::VectorXd & g, const std::vector<int> & offsets, const BlockH & blockH,
  const BlockG & blockG)
{
  const auto n = static_cast<int>(offsets.size());
  for (int i = 0; i < n; i++) {
    g[offsets[i]] += blockG[i];
    for (int j = 0; j < n; j++) {
      H(offsets[i], offsets[j]) += blockH(i, j);
    }
  }
}

/** Offsets of the pose parts of `count` consecutive knots starting at `k`. */
std::vector<int> posePartOffsets(int k, int count, int knotDim)
{
  std::vector<int> offsets;
  offsets.reserve(static_cast<std::size_t>(6 * count));
  for (int c = 0; c < count; c++) {
    const int base = (k + c) * knotDim + kIdxPosition;
    for (int i = 0; i < 6; i++) {
      offsets.push_back(base + i);
    }
  }
  return offsets;
}
}  // namespace

void WindowSystem::resize(int knotCount, int knotDim)
{
  knotCount_ = knotCount;
  knotDim_ = knotDim;
  H_.resize(size(), size());
  g_.resize(size());
  setZero();
}

void WindowSystem::setZero()
{
  H_.setZero();
  g_.setZero();
}

void WindowSystem::addPosePairBlock(int k, const Mat12 & H, const Vec12 & g)
{
  scatter(H_, g_, posePartOffsets(k, 2, knotDim_), H, g);
}

void WindowSystem::addPoseTripletBlock(
  int k, const Eigen::Matrix<double, 18, 18> & H, const Eigen::Matrix<double, 18, 1> & g)
{
  scatter(H_, g_, posePartOffsets(k, 3, knotDim_), H, g);
}

void WindowSystem::addStatePairBlock(int k, const Mat30 & H, const Vec30 & g)
{
  if (knotDim_ != kKnotDim) {
    throw std::runtime_error("addStatePairBlock() needs a window carrying full IMU states");
  }

  std::vector<int> offsets;
  offsets.reserve(2 * kKnotDim);
  for (int c = 0; c < 2; c++) {
    for (int i = 0; i < kKnotDim; i++) {
      offsets.push_back((k + c) * knotDim_ + i);
    }
  }
  scatter(H_, g_, offsets, H, g);
}

void WindowSystem::addStateBlock(int k, const Eigen::MatrixXd & H, const Eigen::VectorXd & g)
{
  const auto n = static_cast<int>(H.rows());
  std::vector<int> offsets;
  offsets.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; i++) {
    offsets.push_back(k * knotDim_ + i);
  }
  scatter(H_, g_, offsets, H, g);
}

void WindowSystem::addLeadingPrior(const Eigen::MatrixXd & H, const Eigen::VectorXd & g)
{
  const auto n = static_cast<int>(H.rows());
  H_.topLeftCorner(n, n) += H;
  g_.head(n) += g;
}

Eigen::VectorXd WindowSystem::solve(double lambda) const
{
  Eigen::MatrixXd A = H_;
  if (lambda > 0) {
    A.diagonal() += lambda * H_.diagonal();
  }
  // A small absolute floor keeps a state that no factor touched from making
  // the whole system singular:
  A.diagonal().array() += 1e-12;

  return A.ldlt().solve(g_);
}

MarginalizationPrior marginalizeLeadingKnots(const WindowSystem & system, int knotsToDrop)
{
  MarginalizationPrior out;

  const int m = knotsToDrop * system.knotDim();
  const int r = system.size() - m;
  if (m <= 0 || r <= 0) {
    return out;
  }

  const auto & H = system.H();
  const auto & g = system.g();

  const Eigen::MatrixXd Hmm = H.topLeftCorner(m, m);
  const Eigen::MatrixXd Hmr = H.topRightCorner(m, r);
  const Eigen::MatrixXd Hrr = H.bottomRightCorner(r, r);
  const Eigen::VectorXd gm = g.head(m);
  const Eigen::VectorXd gr = g.tail(r);

  // The block being dropped can be rank deficient, e.g. a knot no LiDAR
  // segment reached. A pseudo-inverse through the eigendecomposition drops
  // exactly the unobserved directions instead of failing or, worse,
  // manufacturing information along them.
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (Hmm + Hmm.transpose()));
  const Eigen::VectorXd & eigenvalues = es.eigenvalues();

  const double tolerance = 1e-10 * std::max(1.0, eigenvalues.cwiseAbs().maxCoeff());

  Eigen::VectorXd inverseEigenvalues = Eigen::VectorXd::Zero(m);
  for (int i = 0; i < m; i++) {
    if (eigenvalues[i] > tolerance) {
      inverseEigenvalues[i] = 1.0 / eigenvalues[i];
    }
  }
  const Eigen::MatrixXd HmmInv =
    es.eigenvectors() * inverseEigenvalues.asDiagonal() * es.eigenvectors().transpose();

  const Eigen::MatrixXd HrmHmmInv = Hmr.transpose() * HmmInv;

  out.H = Hrr - HrmHmmInv * Hmr;
  out.g = gr - HrmHmmInv * gm;

  // Round-off leaves the Schur complement very slightly asymmetric, which a
  // later Cholesky is entitled to complain about:
  out.H = 0.5 * (out.H + out.H.transpose()).eval();
  out.valid = true;

  return out;
}

}  // namespace mola::ct
