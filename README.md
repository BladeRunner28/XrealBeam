# XrealBeam — IMU probe + anchored virtual screen

Reads the **XREAL Air** IMU over raw USB HID on an Android host (target: Galaxy Note 20 Ultra,
Android 13 / API 33), and renders a world-locked virtual screen on the glasses from that pose.
No vendor SDK — XREAL's own matrix only covers S24/S25, so the raw-HID route is the only route here.

Status 2026-09-14: enumeration + IMU stream + AHRS + render + mount calibration all working on the
device; the open item is verifying that head motion moves the screen the *right way* (see
[Pose path and the mount](#pose-path-and-the-mount)).

## Build

```bash
export JAVA_HOME=/opt/homebrew/opt/openjdk@17          # AGP 9.4 wants JDK 17, not 26
export ANDROID_HOME=$HOME/Library/Android/sdk          # required, or the build aborts
export PATH="$JAVA_HOME/bin:$PATH"
./gradlew assembleDebug
# -> app/build/outputs/apk/debug/app-debug.apk
```

Verified 2026-09-14: AGP 9.4.0 / Gradle 9.7.1 / JDK 17.0.20.1 / compileSdk 36 → BUILD SUCCESSFUL.
`BUILD SUCCESSFUL` does not prove the app code is in the APK — app classes land in `classes3.dex`,
so check the dex (see [Verify off-device](#verify-off-device)).

## Install & run

```bash
$HOME/Library/Android/sdk/platform-tools/adb install -r app/build/outputs/apk/debug/app-debug.apk
adb logcat -s XrealProbe                     # full trace
```

A phone cannot be a USB host and an adb device at the same time, so **install with the glasses
unplugged**, then start the app (it registers the USB-attach receiver, so plugging the glasses in
raises the permission prompt). Plug the glasses in (DeX launches), accept the prompt. **Tap the
screen** to re-run the handshake without reinstalling. Everything the app measures is also written
to `/sdcard/Android/data/com.xman.xrealbeam/files/probe-report.txt`, which survives a replug and is
readable over adb once the glasses come out.

## Pose path and the mount

```
IMU frame → Madgwick AHRS (6-axis) → q_rel = conj(q_ref)·q_cur → change of basis by q_mount
          → world-lock (no filter) or smooth-follow (~0.4 s low-pass) → view matrix = R^T
```

**The mount, measured on the wearer's head (2026-09-14).** Rows are the viewer axes expressed in body
(= IMU) coordinates:

- viewer **right** (+x) → body **−x**
- viewer **up** (+y) → body **+z**
- viewer **back** (+z) → body **+y**

which is a 180° rotation about body (0,1,1)/√2, quaternion **(0, 0, 0.7071, 0.7071)**, and also
reachable with the manual step buttons at mount X 90 / Y 180. Raw inputs: still-pose accel
up = `(−0.004, 0.029, 1.000) g` and forward = `(−0.039, −0.999, 0.029) g`, i.e. within 2.8° of that
clean axis alignment (the residual is how level the wearer's head was, not the hardware).

### How this was got wrong first — read before touching the mount

An earlier build shipped `(0.5, 0.5, 0.5, 0.5)` (up = body **+x**), derived by working backwards from
the wearer's verbal symptom — *"nodding makes it yaw, tilting my head moves the frame up and down"* —
over the 24 proper axis-aligned mounts. That derivation silently assumed which mount the app had been
using when the symptom was observed. **With the prior state unknown, all 24 candidates explain that
sentence** (`tools/derive-mount.py` prints the count), so the "derivation" was a guess in algebraic
clothing, and it came out 90° rolled. It passed the off-device harness, because the harness had been
written to check the pipeline against that same assumption. One minute on the head falsified it.

What is load-bearing instead: the accelerometer reports the world up with a *physical* sign, so two
still poses measure the mount outright, and the wearer's eyes confirm it through the **shape** of each
motion — yaw slides sideways, pitch slides vertically, roll spins in place. Prose can be off by an axis
name (the original report called the spin "yaw"); shapes cannot.

Three other traps, each of which cost a debugging round:

- **The mount is a change of basis**, `q_view = q_mount · q_rel · conj(q_mount)`. A composition
  (`q_mount · q_rel`) stretches motion instead of rotating it.
- **A mount wrong by an axis permutation renders clean, plausible head motion on the wrong axis** —
  which is what made this feel like a rendering bug for a whole day.
- **A single-axis sign flip is not a frame change** — it is a reflection (det −1). The old "yaw sign"
  toggle negated yaw alone in the Euler decomposition, so it flipped pitch and left yaw alone. Sign
  choices are now four *proper* variants (180° conjugations), each flipping two axes, with the mirror
  (about the viewer's up axis) first so one press fixes a mirrored calibration.

### Calibrating the mount on your head (gravity, two still poses)

`CALIBRATE` → prompt appears on the glasses HUD as well as the phone:

1. **Pose 1** — look straight ahead, hold still ~1.5 s. The accelerometer then reads the world up in
   body coordinates: that is the viewer's +y row.
2. **Pose 2** — tilt your head **back** to look straight up, hold still ~1.5 s. Removing the pose-1
   component from this reading leaves the forward axis (exact at any tilt over ~40°; noise scales as
   1/sin t).
3. `right = up × (−forward)` completes the rotation, so the matrix stays proper.
4. **Sign guard** — looking up is a positive rotation about the wearer's right axis, so the gyro
   integral over pose 2 must point along the candidate right axis. Honest limit: this resolves the
   final ambiguity by *trusting the instruction*. The two candidate mounts are indistinguishable from
   accel + gyro alone (they differ by 180° about the vertical, under which both readings are
   unchanged). Tilt forward when asked to look up and you get the mirror mount — visible immediately
   as a nod that reads backwards, fixed by **one** press of `flip` (variant v1).
5. The screen re-centres itself: after the mount is installed the app waits for the next still moment
   and takes that as the LEVEL reference (and restarts the drift clock).

**Pass criterion — the shape test** (in `world-lock`): turning the head must slide the grid
**sideways**, nodding must slide it **vertically**, tilting the head toward a shoulder must **spin** it
in place — three distinct shapes, so the result is one sentence to report. Shapes that come out rotated
(turning slides it up/down, nodding spins it) mean the mount is rolled 90°; a nod that reads backwards
is the mirror and `flip` once fixes it.

`pose now` force-captures the current window as the current pose (for when the glasses wobble on the
nose and the automatic stillness gate will not fire). `RESET` returns to the measured default mount.
`mount X/Y/Z` remain as a manual escape hatch, and the report says which source is in use.

### Frame-rate / drift numbers are in the app

`ahrs` euler angles come from the AHRS in its own z-up world frame; `camera` is what is actually
rendered (viewer frame); the accel line shows the live gravity vector, the window spread used by the
stillness gate, the drift rate, and the ZUPT governor's suppressed yaw.

## Screen geometry, modes and cinema mode

Every size/distance in the app is in metres and the projection is derived from the **optics**, not
picked for looks. The Air shows ~46° diagonal per eye (vendor spec — XREAL's own marketing calls that
"a 130-inch screen from 4 m"), i.e. **40.6° × 23.5°** for a 16:9 panel. The projection used to be 55°
vertically, which magnified the world ~2.4× versus reality and made every size figure fiction.

Modes are expressed as real screen geometry, so they are reproducible and checkable:

- **cinema** — 3.321 m wide at 4.572 m = **150″ at 15 ft** = 39.9° × 23.1° = **98% of the panel**, 48 px/deg
- **desk** — 27″ at 1.6 m = 21.2° = 52% of the panel
- **compact** — 13″ at 1 m = 16.4° = 40% of the panel

Plus `size +/−` (10% steps), `nearer`/`farther` (0.25 m), and a `CLIPPED` flag on the HUD if the
screen is pushed past the optics — the ceiling is the field of view, so "bigger than cinema" means
losing the edges, and the app says so instead of quietly cropping.

**The honest ceiling:** the wearer's cinema ask (150″ at 15 ft = 39.9° wide) and XREAL's advertised
maximum (130″ at 4 m = 39.6° wide) are the same angle to within 0.8%. There is no "even bigger" mode
to add — the optics bound it, and cinema mode is it.

### FOV check (calibrating the one number we did not measure)

46° diagonal is a vendor number. `FOV check` draws a frame at exactly 100% of the assumed FOV, with
brackets at the panel corners: **brackets on the corners** = the model is right, **visible margin** =
the real FOV is wider, **brackets cut off** = narrower. The `FOV` button cycles 46° → 50° → 42° → 52°
so the wearer can pick the one that lines up; the wearer's eyes are the only instrument available for
this, so the check is built in rather than asserted.

## Verify off-device

```bash
./tools/verify-pose.sh        # compiles the shipping HeadPose + MountCal with tools/PoseHarness.java
python3 tools/derive-mount.py # re-derives the default mount from a reported symptom
```

`HeadPose` and `MountCal` deliberately import nothing from Android, so the *shipping* classes run on
the desktop against synthetic ground truth (23 checks, including the screen geometry: the FOV model
round-trips to 46° diagonal, the cinema preset is 3.3207 m at 4.5720 m, it lands at 98.3% of the
panel on both NDC axes, an oversized screen reports CLIPPED, and the size/distance controls clamp). The default mount's rows are checked against
the on-device measurement and must render all six canonical head motions exactly (axis *and* sense);
the four variants must flip the documented axis pairs; the manual steps must reach the same mount at
(90,180,0); `MountCal` must recover a random ground-truth mount from two noisy still poses (worst 2.9°
at 0.01 g per-axis noise over 45–90° tilts, exact for noise-free input), reject a 20° tilt rather than
produce a bad mount, and produce exactly the mirror — fixable with one `flip` press — when the wearer
tilts the wrong way. There is also a **regression guard** that the 90°-rolled mount which shipped by
mistake still fails the shape test; if anyone ever edits `DEFAULT_MOUNT` back toward it, the harness
says so instead of the wearer having to.

```bash
python3 tools/derive-mount.py     # the measurement, the shape test, and why the symptom-derivation
                                  # that produced that rolled mount was not evidence (24/24 candidates)
```

Check the APK really contains the app code:

```bash
python3 - <<'EOF'
import zipfile, os
z = zipfile.ZipFile(os.path.expanduser("~/Projects/XrealBeam/app/build/outputs/apk/debug/app-debug.apk"))
for n in z.namelist():
    if n.endswith(".dex"):
        d = z.read(n)
        if b"Lcom/xman/xrealbeam/HeadPose;" in d:
            print(n, "contains the app classes")
EOF
```

## What it reports

- full interface/endpoint tree of the glasses (so the report is self-documenting)
- `claimInterface(3, force=true)` result — `true` means the kernel `usbhid` driver was detached
- the handshake: each command's byte count, reply sizes, static id, calibration length drained
- after START: frame count, frames/second, INIT-frame count, bad-signature count, read errors
- live gyro / accel / temperature from the parsed frame, plus a hex dump of the first 3 frames
- AHRS angles, drift rate, ZUPT suppression, the mount in use, and the calibration's measured axes

### Probe pass criteria (already met)

1. `claimInterface(3, force=true) -> true`
2. the five commands each report `> 0` bytes written (`OUT-endpoint` or `SET_REPORT` — logged)
3. frames/sec climbs to ~O(100–1000) Hz within a second or two of the START command
4. gyro values move when you physically rotate the glasses, accel ≈ 1 g at rest (measured 489–665 Hz)

## Protocol (ported from thejackimonster/xreal-imu, MIT)

VID `0x3318`; PID `0x0424` = Air, `0x0428` = Air 2, `0x0432` = Air 2 Pro, `0x0426` = Air 2 Ultra.
**IMU interface: 3** on the Air family (2 on the Ultra). MCU interface: 4 (0 on the Ultra).

The stream is **not free-running** — the host must run this handshake on the IMU interface:

| step | msgid | data | then |
|---|---|---|---|
| stop stream | `0x19` | `0x00` | — |
| get static id | `0x1A` | — | read 4 bytes |
| calibration length | `0x14` | — | read 4 bytes = len |
| drain calibration | `0x15` ×N | — | read ≤56 bytes/step until len drained |
| **start stream** | `0x19` | `0x01` | packets flow |

Two device behaviours worth remembering: the first command after opening the device can time out
(expected — badicsalex/ar-drivers-rs #13), and the device answers **every** command with one 64-byte
report whose msgid echoes that command, so reading only after the last command shifts every reply by
one.

### Outbound frame (command)

```
[0] 0xAA | [1..4] CRC32 LE | [5..6] length LE | [7] msgid | [8..] data
length = 3 + len(data)        ; bytes written = 8 + len(data)
CRC32 = standard reflected CRC-32 (init 0xFFFFFFFF, final xor) over length+msgid+data
      = Java's java.util.zip.CRC32 = zlib.crc32   (byte-for-byte matched against the C driver's
        table on 2026-09-14 for lengths 0,1,2,3,4,8,9,12,64)
```

Ground-truth frames (what a working write looks like in a USB trace):

```
STOP            9 bytes  aa 53 e1 26 35 04 00 19 00
GET_STATIC_ID   8 bytes  aa 31 9e 65 00 03 00 1a
GET_CAL_LEN     8 bytes  aa 36 b3 dd e7 03 00 14
CAL_NEXT_SEG    8 bytes  aa a0 83 da 90 03 00 15
START           9 bytes  aa c5 d1 21 42 04 00 19 01
```

### Inbound 64-byte sensor packet

signature `0x01 0x02` = data (`0xAA 0x53` = INIT). Offsets: temp i16 @2 (rate = raw/132.48 + 25,
ICM-42688-P), timestamp u64 @4, gyro mult i16 @12 / div i32 @14, gyro xyz 24-bit signed
@18/21/24, accel mult @27 / div @29, accel xyz @33/36/39, mag mult(byte-swapped) @42 /
div(swapped) @44, mag xyz @48/50/52, checksum @54. Value = `raw * mult / div` (accel in g).
The reference driver runs AHRS at a 1000 Hz sample rate; this port measures ~489 Hz in a plain
blocking read loop.

Accelerometer sign convention (relied on by the mount calibration, verified on-device): at rest the
reading is **+1 g along whichever body axis points up**.

## Still open

- `staticId` read occasionally comes back as a bogus large number (a reply-offset artefact seen in
  one session) — cosmetic, does not affect the stream.
- The factory calibration blob (38,884 bytes of JSON) is captured and saved but not parsed; the
  reference driver and the macOS port both discard it too. Parsing it is unexplored upside for the
  residual yaw drift (steady-state 0.04–0.07°/min warm, ~3°/min cold).
- **Content source is still the procedural grid — this is the next step** (`MediaProjection` →
  `SurfaceTexture`, which touches only the texture, not the pose path). Platform notes already
  settled: single-app window capture needs Android 14 and this phone is 13, so it is full-display
  mirroring; `targetSdk 34` requires a foreground service of type `mediaProjection`; DRM video
  (Netflix/Disney+) renders black by design.
