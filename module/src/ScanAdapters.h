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
 * @file   ScanAdapters.h
 * @brief  MRPT observations to the engine's timed points
 */
#pragma once

#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>

#include <vector>

#include "CtOdometryEngine.h"

namespace mola
{
/** Where a scan's per-point times came from, for diagnostics. */
enum class ScanTimeSource : uint8_t
{
  PerPointRelative,
  PerPointAbsolute,
  AzimuthFallback,
  AzimuthAfterUnusableField,
  SingleInstant
};

[[nodiscard]] const char * toString(ScanTimeSource s);

/** Converts a LiDAR observation into points carrying absolute timestamps, in
 * the body frame.
 *
 * Per-point timing decides everything in a continuous-time method: it is what
 * each sweep is cut into segments on, and a wrong convention does not fail, it
 * just degrades the trajectory. Three cases, in order:
 *
 * 1. **A time field holding scan-relative offsets**, which is what most
 *    drivers publish. Whatever its own convention, starting at zero or
 *    centered on the sweep, it is re-normalized against this scan's own
 *    min/max and anchored on the observation's timestamp.
 * 2. **A time field holding absolute times.** The span comes from the field
 *    itself rather than the observation's stamp, since the two are the same
 *    clock and the field is the more precise of the two.
 * 3. **No usable time field.** Each point's time is synthesized from its
 *    azimuth over `fallbackScanPeriod`, treating the observation's timestamp
 *    as the end of the sweep.
 *
 * A time field that is *present but useless* falls into case 3 too, and
 * catching that matters more here than for a scan-to-map method: a
 * continuous-time estimator handed a sweep of zero duration makes every
 * segment of it degenerate and drifts without ever failing. Two ways it
 * happens, both seen on real data: MRPT holds per-point times as `float`, and
 * a Unix-epoch value near 1.7e9 has a float32 spacing of ~128 s, so a whole
 * sweep quantizes onto one value; and a reader that cannot decode the field's
 * datatype may leave every point at zero. Both are caught by requiring the
 * span to cover at least a few representable steps of the field itself.
 *
 * @param sensorPoseInBody The LiDAR's pose in the body frame. Points are
 *        returned already in the body frame, because that is the frame the
 *        IMU factors live in and everything has to agree on one.
 *
 * @param alreadyDeskewed Set when the provider has already motion-compensated
 *        the cloud onto a single instant. Every point is then given the
 *        observation's own timestamp, whatever the time field says.
 *
 *        This matters more here than anywhere else in the suite. A
 *        provider-deskewed cloud usually *keeps* its per-point time field, so
 *        the times look perfectly usable while the geometry has already been
 *        corrected; placing each point at its own pose would then apply the
 *        correction a second time. The scan is a single-instant observation
 *        and has to be treated as one, which costs this method its
 *        continuous-time advantage on such a dataset but is the only correct
 *        reading of the data.
 */
[[nodiscard]] std::vector<CtOdometryEngine::TimedPoint> toTimedPoints(
  const mrpt::obs::CObservationPointCloud & obs, const mrpt::poses::CPose3D & sensorPoseInBody,
  double fallbackScanPeriod, bool alreadyDeskewed, ScanTimeSource & timeSource);

}  // namespace mola
