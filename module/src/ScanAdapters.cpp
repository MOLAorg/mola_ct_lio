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
 * @file   ScanAdapters.cpp
 * @brief  MRPT observations to the engine's timed points
 */
#include "ScanAdapters.h"

#include <mrpt/maps/CPointsMap.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace mola
{
namespace
{
/// A scan-relative per-point offset spans at most one sweep, so a time field
/// whose smallest value exceeds this is holding absolute times instead. [s]
constexpr double kAbsoluteTimeThreshold = 1e3;

/// How many representable float steps a scan's span must cover for the field
/// to carry usable intra-sweep timing.
constexpr float kMinTimeQuantaPerScan = 16.0f;
}  // namespace

const char * toString(ScanTimeSource s)
{
  switch (s) {
    case ScanTimeSource::PerPointRelative:
      return "per-point relative";
    case ScanTimeSource::PerPointAbsolute:
      return "per-point absolute";
    case ScanTimeSource::AzimuthAfterUnusableField:
      return "azimuth (time field present but unusable)";
    case ScanTimeSource::AzimuthFallback:
    default:
      return "azimuth (no time field)";
  }
}

std::vector<CtOdometryEngine::TimedPoint> toTimedPoints(
  const mrpt::obs::CObservationPointCloud & obs, const mrpt::poses::CPose3D & sensorPoseInBody,
  double fallbackScanPeriod, ScanTimeSource & timeSource)
{
  std::vector<CtOdometryEngine::TimedPoint> out;
  timeSource = ScanTimeSource::AzimuthFallback;

  const auto & pts = obs.pointcloud;
  if (!pts) {
    return out;
  }

  const auto & xs = pts->getPointsBufferRef_x();
  const auto & ys = pts->getPointsBufferRef_y();
  const auto & zs = pts->getPointsBufferRef_z();
  const auto n = xs.size();
  if (n == 0) {
    return out;
  }

  const auto * ts =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  const double obsTime = mrpt::Clock::toDouble(obs.timestamp);

  bool usableTimes = ts && ts->size() == n;
  float tMin = 0;
  float tMax = 0;
  double tBegin = obsTime - fallbackScanPeriod;

  if (usableTimes) {
    tMin = (*ts)[0];
    tMax = (*ts)[0];
    for (size_t i = 0; i < n; i++) {
      tMin = std::min(tMin, (*ts)[i]);
      tMax = std::max(tMax, (*ts)[i]);
    }

    // How far apart two representable values are at this magnitude. A field
    // holding a Unix-epoch time is spaced far wider than a whole sweep, so its
    // intra-sweep timing is simply not in there any more. A reader that failed
    // to decode the field leaves every point equal, which fails the same check.
    const float quantum = std::nextafterf(tMax, std::numeric_limits<float>::infinity()) - tMax;
    const bool spanIsUsable = tMax - tMin > quantum * kMinTimeQuantaPerScan;

    if (!spanIsUsable) {
      timeSource = ScanTimeSource::AzimuthAfterUnusableField;
      usableTimes = false;
    } else if (static_cast<double>(tMin) > kAbsoluteTimeThreshold) {
      timeSource = ScanTimeSource::PerPointAbsolute;
      tBegin = tMin;
    } else {
      timeSource = ScanTimeSource::PerPointRelative;
      tBegin = obsTime + static_cast<double>(tMin);
    }
  }

  const auto & R = sensorPoseInBody.getRotationMatrix();
  const bool identityExtrinsic = sensorPoseInBody == mrpt::poses::CPose3D::Identity();

  out.reserve(n);
  for (size_t i = 0; i < n; i++) {
    CtOdometryEngine::TimedPoint p;

    if (identityExtrinsic) {
      p.p = ct::Vec3(xs[i], ys[i], zs[i]);
    } else {
      const double x = R(0, 0) * xs[i] + R(0, 1) * ys[i] + R(0, 2) * zs[i] + sensorPoseInBody.x();
      const double y = R(1, 0) * xs[i] + R(1, 1) * ys[i] + R(1, 2) * zs[i] + sensorPoseInBody.y();
      const double z = R(2, 0) * xs[i] + R(2, 1) * ys[i] + R(2, 2) * zs[i] + sensorPoseInBody.z();
      p.p = ct::Vec3(x, y, z);
    }

    if (usableTimes) {
      p.t = tBegin + (static_cast<double>((*ts)[i]) - static_cast<double>(tMin));
    } else {
      const double azimuth = std::atan2(static_cast<double>(ys[i]), static_cast<double>(xs[i]));
      const double azimuth01 = (azimuth + M_PI) / (2.0 * M_PI);
      p.t = tBegin + azimuth01 * fallbackScanPeriod;
    }

    out.push_back(p);
  }

  return out;
}

}  // namespace mola
