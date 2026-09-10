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
 * @file   WindowSystem.h
 * @brief  Dense normal equations over a window of control knots
 */
#pragma once

#include <mola_ct_lio/CtSegment.h>
#include <mola_ct_lio/ImuFactor.h>

namespace mola::ct
{
/** The dense normal-equation system over a whole window of control knots.
 *
 * Knot `k` owns the rows and columns `[k * knotDim, (k+1) * knotDim)`, and the
 * pose part always sits at the start of a knot's block, so a pose-only
 * contribution scatters into the same place whether or not the IMU is in use.
 *
 * The window holds a handful of knots, so this is a small dense system and
 * the whole cost of a solve is negligible beside the matching.
 */
class WindowSystem
{
public:
  WindowSystem() = default;
  WindowSystem(int knotCount, int knotDim) { resize(knotCount, knotDim); }

  void resize(int knotCount, int knotDim);
  void setZero();

  [[nodiscard]] int knotCount() const { return knotCount_; }
  [[nodiscard]] int knotDim() const { return knotDim_; }
  [[nodiscard]] int size() const { return knotCount_ * knotDim_; }
  [[nodiscard]] int offsetOf(int knot) const { return knot * knotDim_; }

  /** Adds a contribution spanning the pose parts of knots `k` and `k+1`,
   * ordered `[begin pose(6) ; end pose(6)]`, as produced by the LiDAR
   * segment assembly.
   */
  void addPosePairBlock(int k, const Mat12 & H, const Vec12 & g);

  /** Adds a contribution spanning the full states of knots `k` and `k+1`, as
   * produced by the IMU and bias factors.
   */
  void addStatePairBlock(int k, const Mat30 & H, const Vec30 & g);

  /** Adds a contribution spanning the pose parts of three consecutive knots,
   * ordered `[k(6) ; k+1(6) ; k+2(6)]`, used by the twist-continuity term.
   */
  void addPoseTripletBlock(
    int k, const Eigen::Matrix<double, 18, 18> & H, const Eigen::Matrix<double, 18, 1> & g);

  /** Adds a prior over the leading `H.rows()` variables of the window. */
  void addLeadingPrior(const Eigen::MatrixXd & H, const Eigen::VectorXd & g);

  [[nodiscard]] Eigen::MatrixXd & H() { return H_; }
  [[nodiscard]] const Eigen::MatrixXd & H() const { return H_; }
  [[nodiscard]] Eigen::VectorXd & g() { return g_; }
  [[nodiscard]] const Eigen::VectorXd & g() const { return g_; }

  /** Solves `(H + lambda * diag(H)) x = g`, i.e. the Levenberg-Marquardt form
   * with `lambda = 0` giving plain Gauss-Newton.
   */
  [[nodiscard]] Eigen::VectorXd solve(double lambda) const;

private:
  int knotCount_ = 0;
  int knotDim_ = 0;
  Eigen::MatrixXd H_;
  Eigen::VectorXd g_;
};

/** A quadratic term standing in for the states that have left the window. */
struct MarginalizationPrior
{
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  bool valid = false;

  void clear()
  {
    H.resize(0, 0);
    g.resize(0);
    valid = false;
  }
};

/** Marginalizes the leading `knotsToDrop` knots out of the system, by the
 * Schur complement, leaving a prior over the knots that remain.
 *
 * The result is exact: optimizing the reduced system gives the remaining
 * variables the same values that optimizing the full system would.
 */
[[nodiscard]] MarginalizationPrior marginalizeLeadingKnots(
  const WindowSystem & system, int knotsToDrop);

}  // namespace mola::ct
