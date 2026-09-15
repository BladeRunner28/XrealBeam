# XrealBeam roadmap

Cards, not prose. A card is **done** only when its acceptance test has been run on the device — an
off-device harness pass is evidence, never the last word (see the mount saga in `README.md`).

Status: `done` · `in progress` · `ready` (unblocked, not started) · `research` (unknown feasibility) ·
`blocked` (needs hardware or a decision).

---

## Done

**P1 — Pose pipeline + mount calibration.** AHRS → change-of-basis mount → world-lock/smooth-follow →
view matrix. Mount measured from gravity in two still poses and confirmed by the shape test.
*Acceptance: yaw slides sideways, pitch vertically, roll spins in place.* → 0.4, all three correct on device.

**P2 — Pose modes + head-locked cinema.** world-lock / smooth-follow / head-locked, with cinema
defaulting to head-locked so media is watched fixed to the glasses (the native display behaviour, and
drift becomes irrelevant). *Acceptance: under a 70° head rotation the head-locked camera stays exactly
identity while world-lock still follows.* → done and asserted in the harness (0.5.1).

**G1 — Screen geometry from the optics.** Projection derived from the panel FOV (46° diagonal →
40.6° × 23.5° per eye), metric sizes, presets, `CLIPPED` flag, FOV check frame. *Acceptance: cinema mode
(150″ at 15 ft) measured as 98.3% of panel on both NDC axes; FOV brackets land on the panel corners.*
→ 0.4; the FOV-check confirmation by eye is the one open sub-check.

---

## Tier 1 — content (the current workstream)

**T1.1 — Video file → quad.** `MediaExtractor` + `MediaCodec` decoding into a `SurfaceTexture` bound to
the world-locked quad; play/pause/replay. File chosen through SAF (`ACTION_OPEN_DOCUMENT`, no storage
permission on API 29+).
*Acceptance: pick a local MP4 on the phone, see it playing on the quad, staying put when you turn your head.*
**Status: in progress. Video only — audio is T1.1a.**

**T1.1a — Audio through the glasses — DO THIS NEXT.** `MediaCodec` audio → `AudioTrack`; the Airs
enumerate as USB audio (class 0x01, interfaces 0–2) so their own speakers carry it once they are the
active output. *Acceptance: sound comes out of the glasses, roughly in sync with the image.*
Priority raised: the wearer watches real media (with the light blocker on), and a film with no sound is
not a use case — video-only proved the texture path, which was its job.

**T1.2 — Fill the panel + aspect handling.** `FULL` preset at exactly 100% of the FOV (153″ at 15 ft —
only 2% larger than cinema mode, because cinema mode already is the window), plus FIT / CROP / STRETCH.
*Acceptance: a 2.39:1 file fills the width with the image centred; the letterbox area is see-through, as
the optics have no dimmer; CROP loses the sides by design and says so on the HUD.*

**T1.3 — Mirror the phone display.** `MediaProjection` → `SurfaceTexture` on the same quad, so anything
non-DRM on the phone becomes a floating screen. Needs a foreground service of type `mediaProjection`
(`targetSdk 34`); full-display only, since single-app capture requires Android 14 and this phone is 13.
*Acceptance: phone home screen visible in the quad, updating; a DRM app shows black (expected, documented).*

**T1.4 — Media controls that work with the glasses on.** Face/shoulder gestures for play-pause and
recenter, since reaching for the phone defeats the point. *Acceptance: play/pause and recenter without
touching the phone.*

---

## Tier 2 — immersion

**T2.1 — VR180/360 sphere mode.** Equirectangular video mapped to a sphere around the wearer; rotation-only
3DoF tracking is exactly what this needs (no position tracking required). This is the only mode that
matches "inside the movie".
*Acceptance: a 360 file looks around correctly as the head turns, horizon level, no pole pinching.*
Honest bound: at 40.6° of a 360° panorama you see ~11% at a time — a window into the scene, not a dome
(a Quest is ~110°). **Raised in value by T2.2:** with the light blocker on, that window sits in darkness
rather than in the room, which is what makes a sphere mode read as "inside" instead of "a panel floating
in my kitchen".

**T2.2 — Ambient darkness — SOLVED BY ACCESSORY, no code.** The base Air has no electrochromic dimmer
(an Air 2 Pro feature, 0/35/100%), so a black pixel is a *transparent* pixel: on a combiner you cannot
paint darkness, you can only stop emitting. **The wearer watches media with a clip-on light blocker**,
which removes the ambient light physically — so black really is black, and immersion is bounded only by
the 46° window, not by the room. Consequence for rendering: a "black matte" option would be pointless
(it would not occlude anything); the transparent-area behaviour of FIT is already what a blocker makes
look like black bars. *Acceptance: none needed — record it so nobody re-opens it as a software problem.*

**T2.3 — Curved screen.** Cylindrical quad for a slight wrap-around feel. *Acceptance: curvature visible
and harmless at cinema size.* Low value at this FOV — likely never worth the complexity.

---

## Tier 3 — stereo

**T3.1 — SBS 3D spike.** The Air series switches itself into side-by-side 3D from the glasses' own button
(hold brightness + ~3 s until the chime), so the host only has to render 3840×1080 and each eye takes its
half. Play SBS 3D files and/or render two views.
*Acceptance: hold the button, HUD reports a 3840×1080 display, the two halves line up as a stereo pair.*
Unverified on this Air 1 + firmware — a 10-minute spike decides it.

**T3.2 — MCU interface (HID interface 4) — research.** Never claimed; publicly documented as carrying
control commands (brightness, etc.). Could give programmatic brightness, and possibly SBS if the button
route disappoints. *Acceptance: one documented command acknowledged by the device, with its effect visible.*

---

## Anchoring endurance

**B1 — Parse the factory calibration blob — research.** 38,884 bytes of JSON, discarded by the reference
driver and the macOS port. May carry gyro bias *and* the accelerometer axis alignment — the latter would
independently corroborate the measured mount. *Acceptance: a parsed bias that reduces drift vs the tracked one.*

**B2 — Hour-of-use drift report.** Measure drift with content playing and the head actually moving (the
ZUPT governor only corrects while still). *Acceptance: an hour of use with the screen still acceptably
placed, reported as a number.*

**B3 — Hands-free recenter.** Equivalent of a puck's recenter button: a deliberate gesture (e.g. hold
still looking up, or a double nod) re-zeroes the reference. *Acceptance: recenter possible with the phone
in a pocket.*

---

## Robustness

**D1 — Replug and claim-race hardening.** `claimInterface(3, force=true)` has failed once after a replug
(competing hosts: `ai.nreal.nebula.universal`, `com.xreal.glassescontrol.store`), and hot-plug mid-session
is untested. Retry with backoff, surface the reason, and write the failure modes down.
*Acceptance: unplug/replug 5× in a row and reattach cleanly every time.*

**D2 — DeX takeover behaviour.** DeX may claim the external display before the Presentation; today the
fallback is an in-activity GL surface. *Acceptance: documented behaviour for both paths, no black screen.*
