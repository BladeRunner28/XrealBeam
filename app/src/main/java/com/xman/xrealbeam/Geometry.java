package com.xman.xrealbeam;

import java.util.Locale;

/**
 * Screen geometry and the optics model — the arithmetic that makes "how big does it look" honest.
 *
 * Two things have to agree for a virtual screen to have a truthful angular size:
 *
 * 1. **The projection must match the optics.** The Air shows ~46 deg diagonal per eye (vendor spec;
 *    XREAL's own marketing calls that "a 130-inch screen from 4 m"), i.e. about 40.6 x 23.5 deg for a
 *    16:9 panel. This app used to project 55 deg vertically, which magnifies the world ~2.4x relative
 *    to reality and makes every size/distance figure in the app a fiction. The projection is now
 *    derived from {@link #fovXDeg()} and the actual surface aspect, so a world angle maps to the same
 *    angle in the wearer's eye.
 *
 * 2. **Presets are expressed as real screen geometry**, not percentages: "150-inch at 15 ft" is
 *    3.321 m wide at 4.572 m, which subtends 39.9 x 23.1 deg — 98% of the assumed panel width. That
 *    is essentially the ceiling: XREAL's own advertised maximum (130-inch at 4 m) is 39.6 deg wide.
 *    You cannot have a bigger virtual screen than the optics' field of view, so cinema mode is the
 *    whole window and anything that claims more is clipping.
 *
 * The FOV itself is a vendor number, not something measured here, so {@link #fovCheck} exists: it
 * draws a frame at exactly 100% of the assumed FOV. If the wearer sees margin inside the panel edges,
 * the real FOV is wider than assumed; if the corners are cut off, it is narrower. Their eyes are the
 * only instrument available, so the check is built in rather than assumed.
 *
 * No Android imports: tools/PoseHarness.java runs this on the desktop too.
 */
public class Geometry {

    /** Vendor spec for the Air family (46 deg diagonal); Air 2 Ultra is 52 deg. */
    public static final double AIR_FOV_DIAGONAL_DEG = 46.0;
    private static final double ASPECT = 16.0 / 9.0;

    public enum Mode {
        CINEMA("cinema", 3.321, 4.572, false),   // 150in 16:9 at 15 ft == 39.9 x 23.1 deg, 98% of FOV
        DESK("desk", 0.599, 1.600, false),       // 27in at 1.6 m == 21.2 deg, 52% of FOV
        COMPACT("compact", 0.288, 1.000, false), // 13in at 1 m == 16.4 deg, 40% of FOV
        FULL("full-panel", 0, 4.572, true),      // 153in at 15 ft - 100% of the optics, the ceiling
        CUSTOM("custom", 3.321, 4.572, false);   // whatever the size/distance buttons last produced
        public final String label;
        public final double widthM, distanceM;
        /** True when the width is derived from the field of view rather than a fixed size. */
        public final boolean fillPanel;
        Mode(String label, double widthM, double distanceM, boolean fillPanel) {
            this.label = label;
            this.widthM = widthM;
            this.distanceM = distanceM;
            this.fillPanel = fillPanel;
        }
    }

    /** How source content is mapped onto the screen. */
    public enum Aspect {
        /** Whole frame visible; the screen is shrunk in the constrained axis (letterbox by geometry,
         *  so the unused panel area stays see-through rather than painted black). */
        FIT,
        /** Screen filled; the excess of the source is cropped away (UV range narrowed). */
        CROP,
        /** Whole frame stretched onto the screen, aspect ignored. */
        STRETCH
    }

    public volatile Mode mode = Mode.CINEMA;
    private volatile double widthM = Mode.CINEMA.widthM;
    private volatile double distanceM = Mode.CINEMA.distanceM;
    /** Assumed per-eye field of view, diagonal degrees (vendor spec; tunable for the FOV check). */
    private volatile double fovDiagonalDeg = AIR_FOV_DIAGONAL_DEG;
    /** Draw the full-FOV reference frame over the content, for calibrating the optics assumption. */
    public volatile boolean fovCheck;

    // ---------------- optics ----------------

    public double fovDiagonalDeg() {
        return fovDiagonalDeg;
    }

    public void cycleFov() {
        // 46 is the Air spec; 50 and 42 bracket a panel/spec misstatement in either direction.
        double[] cand = {46.0, 50.0, 42.0, 52.0};
        int i = 0;
        for (int k = 0; k < cand.length; k++) {
            if (Math.abs(cand[k] - fovDiagonalDeg) < 1e-6) {
                i = (k + 1) % cand.length;
                break;
            }
        }
        fovDiagonalDeg = cand[i];
    }

    public double fovXDeg() {
        double t = Math.tan(Math.toRadians(fovDiagonalDeg / 2));
        return 2 * Math.toDegrees(Math.atan(t * ASPECT / Math.hypot(ASPECT, 1)));
    }

    public double fovYDeg() {
        double t = Math.tan(Math.toRadians(fovDiagonalDeg / 2));
        return 2 * Math.toDegrees(Math.atan(t / Math.hypot(ASPECT, 1)));
    }

    // ---------------- screen ----------------

    public double widthM() {
        return mode.fillPanel ? 2 * distanceM * Math.tan(Math.toRadians(fovXDeg() / 2)) : widthM;
    }

    /** Aspect of the loaded source; 16:9 by default so FIT/CROP are no-ops without a video. */
    public volatile double sourceAspect = ASPECT;
    public volatile Aspect aspect = Aspect.FIT;

    public void cycleAspect() {
        Aspect[] all = Aspect.values();
        aspect = all[(aspect.ordinal() + 1) % all.length];
    }

    /** Drawn width after aspect handling. */
    public double contentWidthM() {
        if (aspect != Aspect.FIT) {
            return widthM();
        }
        double w = widthM(), h = w / ASPECT;
        return sourceAspect > ASPECT ? w : h * sourceAspect;
    }

    /** Drawn height after aspect handling. */
    public double contentHeightM() {
        if (aspect != Aspect.FIT) {
            return widthM() / ASPECT;
        }
        double w = widthM(), h = w / ASPECT;
        return sourceAspect > ASPECT ? w / sourceAspect : h;
    }

    /**
     * Texture coordinates to sample for the current aspect mode: {u0, v0, u1, v1}.
     * CROP narrows the UV range (showing the middle of the source across the full screen); FIT does
     * its letterboxing in geometry instead, so it keeps the full range.
     */
    public double[] uvRect() {
        double u0 = 0, v0 = 0, u1 = 1, v1 = 1;
        if (aspect == Aspect.CROP) {
            if (sourceAspect > ASPECT) {           // source wider -> drop the sides
                double f = ASPECT / sourceAspect;
                u0 = (1 - f) / 2;
                u1 = 1 - u0;
            } else {                                // source taller -> drop top and bottom
                double f = sourceAspect / ASPECT;
                v0 = (1 - f) / 2;
                v1 = 1 - v0;
            }
        }
        return new double[]{u0, v0, u1, v1};
    }

    public double heightM() {
        return widthM() / ASPECT;
    }

    public double distanceM() {
        return distanceM;
    }

    public void setMode(Mode m) {
        mode = m;
        widthM = m.widthM;
        distanceM = m.distanceM;
    }

    public void cycleMode() {
        Mode[] all = Mode.values();
        setMode(all[(mode.ordinal() + 1) % all.length]);
    }

    /** Scale the screen in 10% steps, clamped so it cannot invert or vanish. */
    public void scaleSize(double factor) {
        widthM = Math.max(0.05, Math.min(20.0, widthM() * factor));
        mode = Mode.CUSTOM;      // an explicit size is no longer "the panel" or a preset
    }

    /** Push the screen away / pull it closer in 0.25 m steps (0.35 m .. 20 m). */
    public void shiftDistance(double delta) {
        distanceM = Math.max(0.35, Math.min(20.0, distanceM + delta));
    }

    public double angularWidthDeg() {
        // NB: widthM() with parentheses — the raw field is 0 in FULL (panel-filling) mode, and mixing
        // the two is a silent zero here rather than a compile error (caught by the harness).
        return 2 * Math.toDegrees(Math.atan(widthM() / 2 / distanceM));
    }

    public double angularHeightDeg() {
        return 2 * Math.toDegrees(Math.atan(heightM() / 2 / distanceM));
    }

    /** How much of the assumed panel width the screen covers; >100% means the edges are clipped. */
    public double fovFillPercent() {
        return angularWidthDeg() / fovXDeg() * 100.0;
    }

    public double equivalentInches() {
        return widthM() / 0.0254 * Math.hypot(ASPECT, 1) / ASPECT;
    }

    public double equivalentFeet() {
        return distanceM / 0.3048;
    }

    /** True when the screen is wider than the optics can show, i.e. the wearer loses the edges. */
    public boolean clipped() {
        return fovFillPercent() > 100.0;
    }

    /**
     * Build the geometry from a "diagonal inches at feet" spec — the way a wearer thinks about it.
     * Returns {widthM, distanceM} so presets stay self-documenting instead of magic numbers.
     */
    public static double[] fromDiagonalInches(double inches, double feet) {
        double diagM = inches * 0.0254;
        double w = diagM * ASPECT / Math.hypot(ASPECT, 1);
        return new double[]{w, feet * 0.3048};
    }

    public void setDiagonalInches(double inches, double feet) {
        double[] g = fromDiagonalInches(inches, feet);
        widthM = g[0];
        distanceM = g[1];
    }

    public String describe() {
        return String.format(Locale.US, "SCREEN %s %.2fm @%.2fm = %.1fx%.1f deg (%.0f%% FOV, ~%.0fin @%.0fft)%s%s",
                mode.label, widthM(), distanceM, angularWidthDeg(), angularHeightDeg(),
                fovFillPercent(), equivalentInches(), equivalentFeet(),
                clipped() ? " CLIPPED" : "",
                aspect == Aspect.FIT ? "" : " " + aspect);
    }

    public String fovDescribe() {
        return String.format(Locale.US, "FOV %.1f deg diagonal = %.1f x %.1f deg per eye%s",
                fovDiagonalDeg, fovXDeg(), fovYDeg(), fovCheck ? " [check frame ON]" : "");
    }
}
