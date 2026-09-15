package com.xman.xrealbeam;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.media.MediaCodec;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.net.Uri;
import android.util.Log;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.util.Locale;

/**
 * A local video file decoded into a {@link SurfaceTexture} so the world-locked quad can display it.
 *
 * Pipeline: MediaExtractor (samples) -> MediaCodec (hardware decoder) -> Surface -> SurfaceTexture on
 * a GL_TEXTURE_EXTERNAL_OES texture. That is the same texture hand-off MediaProjection will use for
 * mirroring, so the render path is written once.
 *
 * Pacing: the decoder is driven by its own presentation timestamps against a wall clock, re-anchored
 * on every resume. Video only — audio is a separate card (T1.1a), because it doubles the code and the
 * failure modes and this step only has to prove the path. A/V sync therefore does not exist yet.
 *
 * Everything here is deliberately dumb: one video track, no re-buffering, no recovery from a codec
 * reset. The acceptance test is "a file plays on the quad", not "a media player".
 */
public class VideoSource {

    private static final String TAG = "XrealProbe";
    private static final long TIMEOUT_US = 10_000L;

    private MediaExtractor extractor;
    private MediaCodec decoder;
    private Surface surface;
    private Thread thread;
    private volatile boolean stop;
    private volatile long seekToUs = -1;

    // ---- reported state (read by the UI/GL threads) ----
    public volatile String state = "no file";
    public volatile String name = "";
    public volatile int srcWidth, srcHeight, fps, durationMs;
    public volatile long frames, positionUs;
    public volatile boolean playing, eos;
    public volatile long decodeErrors;

    private final float[] stMatrix = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    private long firstPtsUs = -1, startWallNs;

    /** Open a file and start decoding into the given texture. Returns a one-line log for the UI. */
    public String open(Context ctx, Uri uri, SurfaceTexture st) {
        release();
        try {
            extractor = new MediaExtractor();
            extractor.setDataSource(ctx, uri, null);

            int track = -1;
            MediaFormat fmt = null;
            for (int i = 0; i < extractor.getTrackCount(); i++) {
                MediaFormat f = extractor.getTrackFormat(i);
                String mime = f.getString(MediaFormat.KEY_MIME);
                if (mime != null && mime.startsWith("video/")) {
                    track = i;
                    fmt = f;
                    break;
                }
            }
            if (track < 0 || fmt == null) {
                state = "FAILED: no video track in this file";
                return state;
            }
            extractor.selectTrack(track);

            srcWidth = fmt.containsKey(MediaFormat.KEY_WIDTH) ? fmt.getInteger(MediaFormat.KEY_WIDTH) : 0;
            srcHeight = fmt.containsKey(MediaFormat.KEY_HEIGHT) ? fmt.getInteger(MediaFormat.KEY_HEIGHT) : 0;
            fps = fmt.containsKey(MediaFormat.KEY_FRAME_RATE) ? fmt.getInteger(MediaFormat.KEY_FRAME_RATE) : 0;
            durationMs = fmt.containsKey(MediaFormat.KEY_DURATION)
                    ? (int) (fmt.getLong(MediaFormat.KEY_DURATION) / 1000) : 0;

            String mime = fmt.getString(MediaFormat.KEY_MIME);
            decoder = MediaCodec.createDecoderByType(mime);
            surface = new Surface(st);
            decoder.configure(fmt, surface, null, 0);
            decoder.start();

            if (srcWidth > 0 && srcHeight > 0) {
                st.setDefaultBufferSize(srcWidth, srcHeight);
            }
            name = String.valueOf(uri.getLastPathSegment());
            state = "ready";
            eos = false;
            frames = 0;
            decodeErrors = 0;
            positionUs = 0;
            firstPtsUs = -1;
            seekToUs = 0;          // start from the top
            stop = false;
            thread = new Thread(this::loop, "xreal-video");
            thread.setDaemon(true);
            thread.start();
            return String.format(Locale.US, "video %s  %dx%d %dfps %s", name, srcWidth, srcHeight, fps, mmss(durationMs));
        } catch (Exception e) {
            state = "FAILED: " + e;
            Log.w(TAG, "video open failed", e);
            return state;
        }
    }

    public void play() {
        if (eos) {
            replay();
            return;
        }
        firstPtsUs = -1;      // re-anchor the clock so a resume does not fast-forward
        playing = true;
    }

    public void pause() {
        playing = false;
    }

    public void replay() {
        seekToUs = 0;
        eos = false;
        firstPtsUs = -1;
        playing = true;
    }

    public void seekTo(int ms) {
        seekToUs = Math.max(0, ms) * 1000L;
        eos = false;
    }

    /** True once the first frame has been handed to the texture. */
    public boolean ready() {
        return frames > 0 && state != null && !state.startsWith("FAILED");
    }

    public double aspect() {
        return (srcHeight > 0) ? (double) srcWidth / (double) srcHeight : 16.0 / 9.0;
    }

    // ---------------- decode thread ----------------

    private void loop() {
        MediaCodec.BufferInfo bi = new MediaCodec.BufferInfo();
        boolean inputDone = false;
        while (!stop) {
            if (seekToUs >= 0) {
                long t = seekToUs;
                seekToUs = -1;
                try {
                    extractor.seekTo(t, MediaExtractor.SEEK_TO_PREVIOUS_SYNC);
                    decoder.flush();
                    inputDone = false;
                    eos = false;
                    firstPtsUs = -1;
                } catch (Exception e) {
                    decodeErrors++;
                }
            }
            if (!playing) {
                sleep(20);
                continue;
            }
            try {
                if (!inputDone) {
                    int in = decoder.dequeueInputBuffer(TIMEOUT_US);
                    if (in >= 0) {
                        ByteBuffer buf = decoder.getInputBuffer(in);
                        int n = (buf == null) ? -1 : extractor.readSampleData(buf, 0);
                        if (n < 0) {
                            decoder.queueInputBuffer(in, 0, 0, 0, MediaCodec.BUFFER_FLAG_END_OF_STREAM);
                            inputDone = true;
                        } else {
                            decoder.queueInputBuffer(in, 0, n, extractor.getSampleTime(), 0);
                            extractor.advance();
                        }
                    }
                }
                int out = decoder.dequeueOutputBuffer(bi, TIMEOUT_US);
                if (out >= 0) {
                    if ((bi.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) {
                        decoder.releaseOutputBuffer(out, false);
                        eos = true;
                        playing = false;
                        inputDone = false;
                        seekToUs = 0;      // armed for a replay, executed when the user presses play
                        continue;
                    }
                    if (bi.size > 0) {
                        pace(bi.presentationTimeUs);
                        decoder.releaseOutputBuffer(out, true);   // true = render to the Surface
                        frames++;
                        positionUs = bi.presentationTimeUs;
                    } else {
                        decoder.releaseOutputBuffer(out, false);
                    }
                } else if (out == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    MediaFormat nf = decoder.getOutputFormat();
                    if (nf.containsKey(MediaFormat.KEY_WIDTH)) {
                        srcWidth = nf.getInteger(MediaFormat.KEY_WIDTH);
                        srcHeight = nf.getInteger(MediaFormat.KEY_HEIGHT);
                    }
                }
            } catch (IllegalStateException ise) {
                // decoder flushed under us (seek from the UI thread) — loop and pick up the new state
            } catch (Exception e) {
                decodeErrors++;
                if (decodeErrors < 4) {
                    Log.w(TAG, "video decode error: " + e);
                }
                state = "decode error: " + e;
                playing = false;
            }
        }
    }

    /** Sleep until this frame's presentation time, relative to the anchor set on the first frame. */
    private void pace(long ptsUs) {
        if (firstPtsUs < 0) {
            firstPtsUs = ptsUs;
            startWallNs = System.nanoTime();
            return;
        }
        long target = startWallNs + (ptsUs - firstPtsUs) * 1000L;
        while (!stop && playing) {
            long remainNs = target - System.nanoTime();
            if (remainNs <= 0) break;
            sleep(Math.max(1, Math.min(10, remainNs / 1_000_000L)));
        }
    }

    // ---------------- GL side ----------------

    /** Pull the newest decoded frame into the external texture (GL thread only). */
    public void updateTexImage(SurfaceTexture st) {
        try {
            st.updateTexImage();
            st.getTransformMatrix(stMatrix);
        } catch (Exception ignored) {
        }
    }

    public float[] stMatrix() {
        return stMatrix;
    }

    public void release() {
        stop = true;
        playing = false;
        if (thread != null) {
            try { thread.join(500); } catch (InterruptedException ignored) { }
            thread = null;
        }
        try { if (decoder != null) { decoder.stop(); decoder.release(); } } catch (Exception ignored) { }
        decoder = null;
        try { if (surface != null) surface.release(); } catch (Exception ignored) { }
        surface = null;
        try { if (extractor != null) extractor.release(); } catch (Exception ignored) { }
        extractor = null;
        frames = 0;
        state = "no file";
        name = "";
    }

    private static void sleep(long ms) {
        try { Thread.sleep(Math.max(1, ms)); } catch (InterruptedException ignored) { }
    }

    public static String mmss(int ms) {
        int s = Math.max(0, ms / 1000);
        return String.format(Locale.US, "%d:%02d", s / 60, s % 60);
    }

    public String describe() {
        if (!ready()) {
            return "VIDEO " + state;
        }
        return String.format(Locale.US, "VIDEO %s %dx%d @%dfps %s/%s %s%s frames=%d",
                name, srcWidth, srcHeight, fps, mmss((int) (positionUs / 1000)), mmss(durationMs),
                playing ? "PLAYING" : (eos ? "END" : "paused"),
                decodeErrors > 0 ? " errors=" + decodeErrors : "", frames);
    }
}
