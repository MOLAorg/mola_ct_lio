#!/usr/bin/env python3
"""Build a COMFORT localization benchmark archive from raw estimator output.

The benchmark rejects, or silently mis-scores, anything that does not meet
three requirements, and two of them fail quietly:

1. Every row must give `T_world_prism`, not the estimator's own body frame.
   Submitting `base` leaves ATE looking plausible while the relative metrics
   are wrong, because the lever arm is 0.64 m and it rotates with the robot.
2. Every file must be dense. The reference is ~23 Hz and irregular, and the
   scorer associates with a 10 ms tolerance, so a 10 Hz estimate is scored on
   a fifth of its own trajectory and still returns a believable number.
3. The archive holds the six mission files at its root, with no directories.

Usage:
    build-comfort-submission.py <out_dir> <mission>=<tum> [<mission>=<tum> ...]
"""
import json
import os
import subprocess
import sys

import numpy as np

# base -> prism, as x y z yaw pitch roll [m, deg]. The same figure the CI
# dataset map carries for the missions whose reference is the total station.
PRISM = (0.3852, 0.0022, 0.5152, 0.5425, 0.1622, 179.3402)

RATE_HZ = 200.0

REQUIRED = ("arc-2", "arc-7", "con-4", "eig-1", "snow-2", "spx-2")


def ypr_to_R(yaw_deg, pitch_deg, roll_deg):
    y, p, r = np.radians([yaw_deg, pitch_deg, roll_deg])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    return Rz @ Ry @ Rx


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w > 1e-8:
        x = (R[2, 1] - R[1, 2]) / (4 * w)
        y = (R[0, 2] - R[2, 0]) / (4 * w)
        z = (R[1, 0] - R[0, 1]) / (4 * w)
    else:
        # Near a half turn the trace form loses all its precision, so pick the
        # largest diagonal term instead.
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        t = np.sqrt(max(1e-12, 1.0 + R[i, i] - R[j, j] - R[k, k]))
        q = np.zeros(3)
        q[i] = t / 2
        q[j] = (R[j, i] + R[i, j]) / (2 * t)
        q[k] = (R[k, i] + R[i, k]) / (2 * t)
        w = (R[k, j] - R[j, k]) / (2 * t)
        x, y, z = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    return np.array([x, y, z, w]) / n


def slerp(q0, q1, u):
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = np.arccos(d)
    th = th0 * u
    q2 = q1 - q0 * d
    q2 /= np.linalg.norm(q2)
    return q0 * np.cos(th) + q2 * np.sin(th)


def compose_to_prism(d):
    """T_world_base -> T_world_prism, the lever rotated by each pose's own
    orientation. A constant shift will not do: alignment absorbs that, while a
    legged robot pitching under a 0.64 m arm does not."""
    off_t = np.array(PRISM[:3])
    off_R = ypr_to_R(*PRISM[3:])
    out = np.empty_like(d)
    out[:, 0] = d[:, 0]
    for i in range(len(d)):
        R = quat_to_R(d[i, 4:8])
        out[i, 1:4] = d[i, 1:4] + R @ off_t
        out[i, 4:8] = R_to_quat(R @ off_R)
    return out


def densify(d, rate_hz):
    """Resample onto a uniform grid spanning the estimate, SLERP on rotation.

    This adds association, not information: the reference tops out near 23 Hz,
    so the intra-scan gait motion an actual high-rate query would add cannot
    be observed by the metric anyway.
    """
    t0, t1 = d[0, 0], d[-1, 0]
    grid = np.arange(t0, t1 + 0.5 / rate_hz, 1.0 / rate_hz)
    grid = grid[grid <= t1]
    out = np.empty((len(grid), 8))
    out[:, 0] = grid
    for k in range(1, 4):
        out[:, k] = np.interp(grid, d[:, 0], d[:, k])
    idx = np.clip(np.searchsorted(d[:, 0], grid), 1, len(d) - 1)
    for i, g in enumerate(grid):
        j = idx[i]
        ta, tb = d[j - 1, 0], d[j, 0]
        u = 0.0 if tb <= ta else (g - ta) / (tb - ta)
        out[i, 4:8] = slerp(d[j - 1, 4:8], d[j, 4:8], float(np.clip(u, 0.0, 1.0)))
    return out


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    out_dir = sys.argv[1]
    pairs = dict(a.split("=", 1) for a in sys.argv[2:])

    zip_dir = os.path.join(out_dir, "zip")
    os.makedirs(zip_dir, exist_ok=True)

    lever = float(np.linalg.norm(PRISM[:3]))
    report = {"missions": {}, "errors": [], "warnings": []}

    for mission in REQUIRED:
        if mission not in pairs:
            report["errors"].append("%s: no trajectory given" % mission)
            continue
        src = pairs[mission]
        if not os.path.exists(src):
            report["errors"].append("%s: %s does not exist" % (mission, src))
            continue

        raw = np.loadtxt(src)
        if raw.ndim != 2 or raw.shape[1] != 8:
            report["errors"].append("%s: expected 8 columns, got %s" % (mission, raw.shape))
            continue

        prism = compose_to_prism(raw)
        dense = densify(prism, RATE_HZ)

        # The composed track must sit exactly one lever arm from the body
        # track at every pose; anything else means the rotation was applied
        # wrongly, which is the failure this check exists to catch.
        worst = float(np.abs(np.linalg.norm(prism[:, 1:4] - raw[:, 1:4], axis=1) - lever).max())

        dst = os.path.join(zip_dir, "%s.tum" % mission)
        np.savetxt(dst, dense, fmt="%.17g")

        span = float(dense[-1, 0] - dense[0, 0])
        entry = {
            "source": os.path.abspath(src),
            "lever_check_m": worst,
            "poses": int(len(dense)),
            "span_s": span,
            "rate_hz": round(len(dense) / span, 3) if span > 0 else 0.0,
        }
        report["missions"][mission] = entry

        if worst > 1e-4:
            report["errors"].append("%s: lever check %.3g m, the compose is wrong" % (mission, worst))
        if len(dense) < 100:
            report["errors"].append("%s: only %d poses" % (mission, len(dense)))
        if not np.all(np.diff(dense[:, 0]) > 0):
            report["errors"].append("%s: timestamps are not strictly increasing" % mission)
        if not np.isfinite(dense).all():
            report["errors"].append("%s: non-finite values" % mission)

    zip_path = os.path.join(out_dir, "comfort_submission.zip")
    if not report["errors"]:
        if os.path.exists(zip_path):
            os.remove(zip_path)
        files = [os.path.join(zip_dir, "%s.tum" % m) for m in REQUIRED]
        subprocess.run(["zip", "-j", "-q", zip_path] + files, check=True)
        report["zip"] = zip_path
        report["zip_bytes"] = os.path.getsize(zip_path)

    report["ok"] = not report["errors"]
    with open(os.path.join(out_dir, "precheck.json"), "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report, indent=1))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
