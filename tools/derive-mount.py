#!/usr/bin/env python3
"""
How the default body->viewer mount was determined — including the way it was got WRONG first.

This file exists so nobody (including a future session) re-derives the mount from a symptom sentence
and ships a 90-degree-rolled answer, which is what happened on 2026-09-14. Run it with:

    python3 tools/derive-mount.py

It prints, in order:
  1. the mount MEASURED on the wearer's head (two still poses, gravity) — the actual truth
  2. the strict shape test for that mount (all five canonical head motions, axis AND sense)
  3. why the earlier symptom-derived answer was not evidence (24/24 candidates fit)
  4. the regression: the rolled mount that shipped by mistake fails the shape test
"""
import itertools
import numpy as np

D2R = np.pi / 180.0

# ---------------------------------------------------------------- the on-device measurement
# Raw accelerometer vectors from probe-report.txt (worn, Note 20 Ultra + XREAL Air, 2026-09-14):
#   pose 1: still, looking level   -> accel = the world up, in body coords
#   pose 2: still, head tilted back-> accel = up*cos(t) + forward*sin(t)   (t = 49 deg)
UP_BODY = np.array([-0.0042, 0.0291, 0.9996])      # g
FWD_BODY = np.array([-0.0385, -0.9988, 0.0289])    # g

# The clean axis alignment this measurement sits within 3 deg of (the residual is how level the
# wearer's head was, not the hardware). Rows = viewer axes expressed in body coordinates.
MOUNT_ROWS = np.array([[-1, 0, 0],   # viewer +x (right) -> body -x
                       [0, 0, 1],    # viewer +y (up)    -> body +z
                       [0, 1, 0]])   # viewer +z (back)  -> body +y


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qn(q):
    return q / np.linalg.norm(q)


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
        return qn(np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                            (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]))
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = np.zeros(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return qn(q)


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


def from_yxz(y, p, r):
    return qmul(np.array([np.cos(y / 2), 0, np.sin(y / 2), 0]),
                qmul(np.array([np.cos(p / 2), np.sin(p / 2), 0, 0]),
                     np.array([np.cos(r / 2), 0, 0, np.sin(r / 2)])))


def render(q_body, q_mount, yaw_flip=False):
    """The app's render path (reference at identity). yaw_flip = the old improper sign toggle."""
    q = qmul(q_mount, qmul(q_body, qconj(q_mount)))
    if yaw_flip:
        y, p, r = to_yxz(q)
        q = from_yxz(-y, p, r)
    return q


def axis_of(q):
    q = qn(q)
    v = q[1:]
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else np.zeros(3)


MOTIONS = {"turn LEFT": (0, 1, 0), "look UP (nod)": (1, 0, 0), "look DOWN (nod)": (-1, 0, 0),
           "tilt RIGHT": (0, 0, -1), "tilt LEFT": (0, 0, 1)}


def shape(q):
    """Which motion shape the rendered rotation has: yaw = about up, pitch = about right,
    roll = about forward. This is the wearer's acceptance test in one word."""
    a = axis_of(q)
    k = int(np.argmax(np.abs(a)))
    return ["pitch", "yaw", "roll"][k], a


def main():
    up = UP_BODY / np.linalg.norm(UP_BODY)
    fwd = FWD_BODY / np.linalg.norm(FWD_BODY)
    back = -fwd
    right = np.cross(up, back)
    measured = np.vstack([right, up, back])
    q_measured = q_of_R(measured)
    q_truth = q_of_R(MOUNT_ROWS)
    err = np.degrees(np.arccos(np.clip((np.trace(measured @ MOUNT_ROWS.T) - 1) / 2, -1, 1)))

    print("=== 1. the measurement (what actually determines the mount) ===")
    print("worn up (body) :", np.round(up, 4).tolist(), "g")
    print("forward (body) :", np.round(fwd, 4).tolist(), "g")
    print("=> mount rows  :", MOUNT_ROWS.tolist())
    print("   measurement is %.2f deg from that clean alignment" % err)
    print("   quaternion   :", np.round(q_truth, 6).tolist(),
          "= 180 deg about body", np.round(axis_of(q_truth), 4).tolist())
    print("   step buttons : reachable at mount X 90 / Y 180")

    print("\n=== 2. the acceptance test: shape of each motion (no algebra required) ===")
    worst = 1.0
    for name, ax in MOTIONS.items():
        q_c = axis_angle(ax, 30.0)
        q_b = qmul(qconj(q_truth), qmul(q_c, q_truth))     # same motion in gyro/body coords
        out = render(q_b, q_truth)
        dot = abs(float(np.dot(qn(out), q_c)))
        worst = min(worst, dot)
        print(f"  {name:15s} -> {shape(out)[0]:5s}  |<q_out,q_intended>| = {dot:.6f}")
    print("  ->", "PASS" if worst > 1 - 1e-9 else "FAIL")

    print("\n=== 3. why a symptom sentence is NOT evidence ===")
    print("The wearer reported: 'nodding makes it yaw, tilting my head moves the frame up and down'.")
    print("That constrains things only if you know which mount the app was using at the time.")
    reported = {"look UP (nod)": "yaw", "tilt RIGHT": "pitch"}
    per_truth = {}
    all_mounts = []
    for p in itertools.permutations(range(3)):
        for sg in itertools.product([1, -1], repeat=3):
            m = np.zeros((3, 3))
            for row, (col, s) in enumerate(zip(p, sg)):
                m[row, col] = s
            if np.linalg.det(m) > 0.5:
                all_mounts.append(m)
    for truth in all_mounts:
        qt = q_of_R(truth)
        fits = 0
        for sx, sy, sz in itertools.product(range(4), repeat=3):
            qc = np.array([1.0, 0, 0, 0])
            for ax, st in enumerate((sx, sy, sz)):
                qc = qmul(qc, stepQ(ax, st))
            qc = qn(qc)
            for flip in (True, False):
                ok = True
                for motion, seen in reported.items():
                    q_c = axis_angle(MOTIONS[motion], 30.0)
                    q_b = qmul(qconj(qt), qmul(q_c, qt))
                    got = axis_of(render(q_b, qc, flip))
                    want = 1 if seen == "yaw" else 0
                    if abs(got[want]) < 0.99:
                        ok = False
                        break
                if ok:
                    fits += 1
        per_truth[tuple(truth.astype(int).flatten().tolist())] = fits
    fits_all = sum(1 for v in per_truth.values() if v > 0)
    print(f"  candidate mounts that can explain it: {fits_all}/24  (median "
          f"{int(np.median(list(per_truth.values())))} old-mount states each)")
    print("  => with the app's prior state unknown the sentence eliminates nothing, so a 'derivation'")
    print("     from it is a guess. The 90-deg-rolled default that shipped came out of exactly that.")

    print("\n=== 4. regression: the rolled default that shipped by mistake ===")
    rolled = np.array([0.5, 0.5, 0.5, 0.5])
    bad = 0
    for name, ax in MOTIONS.items():
        q_c = axis_angle(ax, 30.0)
        q_b = qmul(qconj(q_truth), qmul(q_c, q_truth))     # the real head motion, in body coords
        out = render(q_b, rolled)                          # ...rendered by the rolled mount
        same = abs(float(np.dot(qn(out), q_c))) > 1 - 1e-9
        bad += 0 if same else 1
        print(f"  {name:15s} -> {shape(out)[0]:5s} (want {shape(q_c)[0]:5s})"
              f"   {'ok' if same else 'WRONG'}")
    print(f"  {bad}/5 motions wrong; it was falsified on the head in about a minute.")


if __name__ == "__main__":
    main()
