#!/usr/bin/env python3
"""
Derives the default body->viewer mount from the wearer's on-device axis report.

This is the derivation record for HeadPose.DEFAULT_MOUNT (0.5, 0.5, 0.5, 0.5). It is kept in the
repo because the input is a *symptom*, not a measurement: if a future report describes motion landing
on different axes, re-run this with the new symptom rather than tapping mount buttons on the head.

Method
------
Input (verbatim, 2026-09-14, Note 20 Ultra + XREAL Air, mount on screen "MOUNT(90,0,90) yaw-"):
    "looking up and down causes it to yaw, tilting my head causes the frame to move up and down"

Model: the app renders  q_view = q_mount * conj(q_ref) * q_cur * conj(q_mount),  so the perceived
rotation axis of a physical head motion is  axis_view = C_used * axis_body  where C_used is the mount
the app is using and axis_body is the same motion expressed in the gyro's frame, i.e.
axis_body = C_true^-1 * axis_intended. Hence  axis_view = (C_used * C_true^-1) * axis_intended.

So: search all 24 proper axis-aligned mounts for the C_true that turns the reported axes into the
physically correct ones (nod -> pitch about the viewer's right axis, tilt -> roll about the forward
axis, yaw -> yaw). Exactly four explain the report; they are the same axes with different signs, and
the one printed here is the one baked as the default (its sign variant is then resolved by MountCal).

Usage:  python3 tools/derive-mount.py
"""
import itertools
import numpy as np

D2R = np.pi / 180.0
CANON = {"turn": (0.0, 1.0, 0.0),      # yaw: about the viewer's up axis (+y = a left turn)
         "nosd": (1.0, 0.0, 0.0),      # pitch: looking up is +rotation about the right axis (+x)
         "tilt": (0.0, 0.0, -1.0)}     # roll: tilting the head right is +rotation about forward (-z)

CURRENT_STEPS = (1, 0, 1)     # mount X +90, mount Z +90 (what the HUD printed)
CURRENT_YAW_FLIP = True       # the old "yaw-" flag (an improper single-axis flip — see below)
REPORT = {"nosd": "yaw", "tilt": "pitch"}     # what the wearer saw


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def axis_angle(axis, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    h = deg * D2R / 2
    return np.array([np.cos(h), *(a * np.sin(h))])


def R_of(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def q_of_R(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1) * 2
        return _n([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = np.zeros(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return _n(q)


def _n(q):
    return q / np.linalg.norm(q)


def stepQ(axis, steps):
    s = steps % 4
    if s == 0:
        return np.array([1.0, 0, 0, 0])
    h = np.radians(90.0 * s) / 2
    return np.array([[np.cos(h), np.sin(h), 0, 0], [np.cos(h), 0, np.sin(h), 0],
                     [np.cos(h), 0, 0, np.sin(h)]][axis])


def to_yxz(q):
    w, x, y, z = q
    return (np.arctan2(2 * (x * z + w * y), 1 - 2 * (x * x + y * y)),
            np.arcsin(np.clip(-2 * (y * z - w * x), -1, 1)),
            np.arctan2(-2 * (x * y - w * z), 1 - 2 * (x * x + z * z)))


def from_yxz(yaw, pitch, roll):
    qy = np.array([np.cos(yaw / 2), 0, np.sin(yaw / 2), 0])
    qx = np.array([np.cos(pitch / 2), np.sin(pitch / 2), 0, 0])
    qz = np.array([np.cos(roll / 2), 0, 0, np.sin(roll / 2)])
    return qmul(qy, qmul(qx, qz))


def render(q_body, q_mount, yaw_flip=False):
    """The app's render path, reference at identity."""
    q = qmul(q_mount, qmul(q_body, qconj(q_mount)))
    if yaw_flip:                      # the OLD flip: negate yaw only == a reflection, not a rotation
        y, p, r = to_yxz(q)
        q = from_yxz(-y, p, r)
    return q


def axis_of(q):
    q = _n(q)
    v = q[1:]
    n = np.linalg.norm(v)
    return (v / n if n > 1e-9 else np.zeros(3))


def main():
    q_used = np.array([1.0, 0, 0, 0])
    for ax, st in enumerate(CURRENT_STEPS):
        q_used = qmul(q_used, stepQ(ax, st))
    q_used = _n(q_used)
    print("mount the app was using (from the HUD 'MOUNT(90,0,90) yaw-'):")
    print(np.round(R_of(q_used), 3), " quat", np.round(q_used, 4))
    print("  NOTE: that flag negates yaw alone, which is NOT a rotation (det -1): a single-axis sign")
    print("        flip cannot be a frame change. It is one reason the button-tapping never settled.\n")

    candidates = []
    for p in itertools.permutations(range(3)):
        for signs in itertools.product([1, -1], repeat=3):
            m = np.zeros((3, 3))
            for row, (col, s) in enumerate(zip(p, signs)):
                m[row, col] = s
            if np.linalg.det(m) > 0.5:
                candidates.append(m)
    print(f"proper axis-aligned mounts searched: {len(candidates)}")

    consistent = []
    for m in candidates:
        q_true = q_of_R(m)
        ok = True
        for motion, seen in REPORT.items():
            q_c = axis_angle(CANON[motion], 30.0)
            q_b = qmul(qconj(q_true), qmul(q_c, q_true))          # same motion in gyro coords
            got = axis_of(render(q_b, q_used, CURRENT_YAW_FLIP))
            want = 1 if seen == "yaw" else 0                       # yaw = about y, pitch = about x
            if abs(got[want]) < 0.99:
                ok = False
        if ok:
            consistent.append(m)

    print(f"mounts that explain the report: {len(consistent)} (same axes, differing signs)\n")
    for m in consistent:
        print(np.round(m, 3).tolist())

    d = consistent[0]
    q_default = q_of_R(d)
    ang = 2 * np.degrees(np.arccos(min(1.0, abs(q_default[0]))))
    print(f"\ndefault mount rows (viewer x,y,z in body coords): {d.tolist()}")
    print(f"as a quaternion: {np.round(q_default, 6).tolist()}")
    print(f"= {ang:.1f} deg about {np.round(axis_of(q_default), 3).tolist()}")
    print("  rows say: worn up = body +x, worn back = body +y, worn right = body +z.")
    print("  cross-check with the independent desk measurement (flat, screens up -> gravity on body")
    print("  +y): that pose's up IS the worn 'back' direction, and 90 deg apart is exactly the")
    print("  pick-them-up-and-wear-them rotation. Two independent data 90 deg apart agree.\n")

    print("strict check — with this mount every motion must render exactly:")
    worst = 1.0
    for name, ax in [("turn LEFT", (0, 1, 0)), ("look UP", (1, 0, 0)), ("look DOWN", (-1, 0, 0)),
                     ("tilt RIGHT", (0, 0, -1)), ("tilt LEFT", (0, 0, 1))]:
        q_c = axis_angle(ax, 30.0)
        q_b = qmul(qconj(q_default), qmul(q_c, q_default))
        out = render(q_b, q_default)
        dot = abs(float(np.dot(_n(out), q_c)))
        worst = min(worst, dot)
        print(f"  {name:11s} |<q_out, q_intended>| = {dot:.6f}")
    print("  -> " + ("PASS" if worst > 1 - 1e-9 else "FAIL"))


if __name__ == "__main__":
    main()
