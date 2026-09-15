package com.xman.xrealbeam;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.opengl.GLSurfaceView;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Renders the virtual screen: one textured quad at the geometry's distance, viewed through the head
 * pose. The texture is either a local video file (decoded into a {@link SurfaceTexture}, i.e. an
 * external texture) or the procedural grid used to judge the pose path when no file is loaded.
 *
 * Two coordinate systems live here and mixing them is the classic bug:
 *   - the world quad: model = translate(0,0,-distance) * scale(contentWidth, contentHeight),
 *     camera = head pose, projection = the optics' field of view (from {@link Geometry});
 *   - the FOV check overlay: drawn in normalized device coordinates with an identity MVP, so it
 *     always sits exactly on the panel edge regardless of pose.
 *
 * Aspect handling is split on purpose: FIT is done in geometry (a smaller quad, so the unused panel
 * area stays see-through rather than painted black — these optics have no dimmer), CROP is done in
 * texture coordinates (sampling the middle of the source across the full quad).
 */
public class ScreenRenderer implements GLSurfaceView.Renderer {

    // ---- pattern program (2D texture) ----
    private static final String VS_PATTERN =
            "uniform mat4 uMvp;\n" +
            "uniform vec4 uUvRect;\n" +
            "uniform float uFlipY;\n" +
            "attribute vec4 aPos;\n" +
            "attribute vec2 aUv;\n" +
            "varying vec2 vUv;\n" +
            "void main() {\n" +
            "  vec2 uv = mix(uUvRect.xy, uUvRect.zw, vec2(aUv.x, mix(aUv.y, 1.0 - aUv.y, uFlipY)));\n" +
            "  vUv = uv;\n" +
            "  gl_Position = uMvp * aPos;\n" +
            "}\n";

    private static final String FS_PATTERN =
            "precision mediump float;\n" +
            "uniform sampler2D uTex;\n" +
            "varying vec2 vUv;\n" +
            "void main() {\n" +
            "  gl_FragColor = texture2D(uTex, vUv);\n" +
            "}\n";

    // ---- video program (external texture, with the SurfaceTexture transform matrix) ----
    private static final String VS_VIDEO =
            "uniform mat4 uMvp;\n" +
            "uniform vec4 uUvRect;\n" +
            "uniform mat4 uSt;\n" +
            "uniform float uFlipY;\n" +
            "attribute vec4 aPos;\n" +
            "attribute vec2 aUv;\n" +
            "varying vec2 vUv;\n" +
            "void main() {\n" +
            "  vec2 uv = mix(uUvRect.xy, uUvRect.zw, vec2(aUv.x, mix(aUv.y, 1.0 - aUv.y, uFlipY)));\n" +
            "  vUv = (uSt * vec4(uv, 0.0, 1.0)).xy;\n" +
            "  gl_Position = uMvp * aPos;\n" +
            "}\n";

    private static final String FS_VIDEO =
            "#extension GL_OES_EGL_image_external : require\n" +
            "precision mediump float;\n" +
            "uniform samplerExternalOES uTex;\n" +
            "varying vec2 vUv;\n" +
            "void main() {\n" +
            "  gl_FragColor = texture2D(uTex, vUv);\n" +
            "}\n";

    /** Unit quad: scaled per frame from the geometry (so size/distance are live). */
    private static final float[] QUAD = {
            -0.5f, -0.5f, 0f,
             0.5f, -0.5f, 0f,
            -0.5f,  0.5f, 0f,
             0.5f,  0.5f, 0f,
    };
    private static final float[] UV = {0f, 1f, 1f, 1f, 0f, 0f, 1f, 0f};
    /** Full-frame quad in NDC, for the FOV check overlay (identity MVP). */
    private static final float[] NDC_QUAD = {
            -1f, -1f, 0f,
             1f, -1f, 0f,
            -1f,  1f, 0f,
             1f,  1f, 0f,
    };

    private final HeadPose pose;
    private final XrealImu imu;
    private final Geometry geom;
    /** Set by the Activity once a file is opened; the renderer only reads it. */
    public volatile VideoSource video;

    private Prog pattern, videoProg;
    private int textureId, overlayId, videoTexId = -1;
    private SurfaceTexture videoSt;
    private final float[] proj = new float[16];
    private final float[] view = new float[16];
    private final float[] model = new float[16];
    private final float[] mvp = new float[16];
    private final float[] scratch = new float[16];
    private final float[] identity = new float[16];
    private final float[] uvRect = new float[4];
    private long lastFrameNs;
    private int width = 1920, height = 1080;

    public ScreenRenderer(HeadPose pose, XrealImu imu, Geometry geom) {
        this.pose = pose;
        this.imu = imu;
        this.geom = geom;
    }

    /** The texture the video decoder writes into. Null until the GL context exists. */
    public SurfaceTexture videoSurfaceTexture() {
        return videoSt;
    }

    /** A tiny program plus its uniform locations. */
    private static final class Prog {
        int program, aPos, aUv, uMvp, uTex, uUvRect, uFlipY;
        Prog(int p, boolean withStMatrixPlaceholder) {
            program = p;
            aPos = GLES20.glGetAttribLocation(p, "aPos");
            aUv = GLES20.glGetAttribLocation(p, "aUv");
            uMvp = GLES20.glGetUniformLocation(p, "uMvp");
            uTex = GLES20.glGetUniformLocation(p, "uTex");
            uUvRect = GLES20.glGetUniformLocation(p, "uUvRect");
            uFlipY = GLES20.glGetUniformLocation(p, "uFlipY");
        }
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        pattern = new Prog(link(VS_PATTERN, FS_PATTERN), false);
        videoProg = new Prog(link(VS_VIDEO, FS_VIDEO), true);
        textureId = createTexture(buildTestPattern());
        overlayId = createTexture(buildFovCheckFrame());
        videoTexId = createExternalTexture();
        if (videoTexId > 0) {
            videoSt = new SurfaceTexture(videoTexId);
            videoSt.setDefaultBufferSize(1920, 1080);
        }
        GLES20.glClearColor(0.02f, 0.02f, 0.04f, 1f);
        GLES20.glDisable(GLES20.GL_DEPTH_TEST);
        for (int i = 0; i < 16; i++) identity[i] = 0f;
        identity[0] = identity[5] = identity[10] = identity[15] = 1f;
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int w, int h) {
        width = Math.max(1, w);
        height = Math.max(1, h);
        GLES20.glViewport(0, 0, width, height);
        setProjection();
    }

    /**
     * Projection straight from the optics model: horizontal FOV is the anchor, the vertical FOV
     * follows from the surface's own aspect, so a world angle lands as the same angle in the eye
     * whatever the panel size turns out to be.
     */
    private void setProjection() {
        double aspect = (double) width / (double) height;
        double fovy = 2 * Math.toDegrees(Math.atan(Math.tan(Math.toRadians(geom.fovXDeg() / 2)) / aspect));
        perspective(proj, (float) fovy, (float) aspect, 0.05f, 200f);
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        long now = System.nanoTime();
        double dt = lastFrameNs == 0 ? 1.0 / 60.0 : (now - lastFrameNs) / 1e9;
        lastFrameNs = now;
        pose.update(imu.quatW, imu.quatX, imu.quatY, imu.quatZ, dt);
        pose.getViewMatrix(view);

        // World quad: metric content size from the geometry, placed at the distance.
        float w = (float) geom.contentWidthM();
        float h = (float) geom.contentHeightM();
        float d = (float) geom.distanceM();
        scaleTranslate(model, w, h, -d);
        multiply(scratch, view, model);
        multiply(mvp, proj, scratch);

        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);

        double[] r = geom.uvRect();
        uvRect[0] = (float) r[0];
        uvRect[1] = (float) r[1];
        uvRect[2] = (float) r[2];
        uvRect[3] = (float) r[3];

        VideoSource v = video;
        boolean showVideo = v != null && v.ready() && videoSt != null && videoTexId > 0;
        if (showVideo) {
            v.updateTexImage(videoSt);
            Prog p = videoProg;
            GLES20.glUseProgram(p.program);
            GLES20.glUniform4fv(p.uUvRect, 1, uvRect, 0);
            GLES20.glUniform1f(p.uFlipY, 1f);      // MediaCodec surfaces are v-flipped vs GL
            int uSt = GLES20.glGetUniformLocation(p.program, "uSt");
            GLES20.glUniformMatrix4fv(uSt, 1, false, v.stMatrix(), 0);
            drawQuad(QUAD, GLES11Ext.GL_TEXTURE_EXTERNAL_OES, videoTexId, p, mvp);
        } else {
            Prog p = pattern;
            GLES20.glUseProgram(p.program);
            GLES20.glUniform4fv(p.uUvRect, 1, uvRect, 0);
            GLES20.glUniform1f(p.uFlipY, 0f);
            drawQuad(QUAD, GLES20.GL_TEXTURE_2D, textureId, p, mvp);
        }

        // FOV check: full-frame overlay in NDC, unaffected by the pose.
        if (geom.fovCheck) {
            GLES20.glEnable(GLES20.GL_BLEND);
            GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);
            Prog p = pattern;
            GLES20.glUseProgram(p.program);
            GLES20.glUniform4fv(p.uUvRect, 1, FULL_UV, 0);
            GLES20.glUniform1f(p.uFlipY, 0f);
            drawQuad(NDC_QUAD, GLES20.GL_TEXTURE_2D, overlayId, p, identity);
            GLES20.glDisable(GLES20.GL_BLEND);
        }
    }

    private static final float[] FULL_UV = {0f, 0f, 1f, 1f};

    private void drawQuad(float[] verts, int target, int tex, Prog p, float[] matrix) {
        FloatBuffer vb = buf(verts);
        FloatBuffer tb = buf(UV);
        GLES20.glEnableVertexAttribArray(p.aPos);
        GLES20.glVertexAttribPointer(p.aPos, 3, GLES20.GL_FLOAT, false, 0, vb);
        GLES20.glEnableVertexAttribArray(p.aUv);
        GLES20.glVertexAttribPointer(p.aUv, 2, GLES20.GL_FLOAT, false, 0, tb);
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
        GLES20.glBindTexture(target, tex);
        GLES20.glUniform1i(p.uTex, 0);
        GLES20.glUniformMatrix4fv(p.uMvp, 1, false, matrix, 0);
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);
        GLES20.glDisableVertexAttribArray(p.aPos);
        GLES20.glDisableVertexAttribArray(p.aUv);
    }

    public String sizeString() {
        return width + "x" + height;
    }

    // ---------------- helpers ----------------

    private static FloatBuffer buf(float[] a) {
        ByteBuffer bb = ByteBuffer.allocateDirect(a.length * 4).order(ByteOrder.nativeOrder());
        FloatBuffer fb = bb.asFloatBuffer();
        fb.put(a).position(0);
        return fb;
    }

    private static int link(String vsSrc, String fsSrc) {
        int vs = compile(GLES20.GL_VERTEX_SHADER, vsSrc);
        int fs = compile(GLES20.GL_FRAGMENT_SHADER, fsSrc);
        int p = GLES20.glCreateProgram();
        GLES20.glAttachShader(p, vs);
        GLES20.glAttachShader(p, fs);
        GLES20.glLinkProgram(p);
        int[] ok = new int[1];
        GLES20.glGetProgramiv(p, GLES20.GL_LINK_STATUS, ok, 0);
        if (ok[0] == 0) {
            throw new RuntimeException("program link failed: " + GLES20.glGetProgramInfoLog(p));
        }
        return p;
    }

    private static int compile(int type, String src) {
        int s = GLES20.glCreateShader(type);
        GLES20.glShaderSource(s, src);
        GLES20.glCompileShader(s);
        int[] ok = new int[1];
        GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0);
        if (ok[0] == 0) {
            throw new RuntimeException("shader compile failed: " + GLES20.glGetShaderInfoLog(s));
        }
        return s;
    }

    private static int createExternalTexture() {
        int[] ids = new int[1];
        GLES20.glGenTextures(1, ids, 0);
        if (ids[0] == 0) return -1;
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, ids[0]);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
        return ids[0];
    }

    /** High-contrast grid, corner markers and a crosshair so a wrong axis is obvious. */
    private static Bitmap buildTestPattern() {
        int w = 1280, h = 720;
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        Canvas c = new Canvas(bmp);
        c.drawColor(Color.rgb(12, 14, 20));

        Paint grid = new Paint();
        grid.setColor(Color.rgb(45, 55, 75));
        grid.setStrokeWidth(2f);
        for (int x = 0; x <= w; x += 80) c.drawLine(x, 0, x, h, grid);
        for (int y = 0; y <= h; y += 80) c.drawLine(0, y, w, y, grid);

        Paint border = new Paint();
        border.setColor(Color.rgb(90, 170, 255));
        border.setStyle(Paint.Style.STROKE);
        border.setStrokeWidth(10f);
        c.drawRect(5, 5, w - 5, h - 5, border);

        Paint cross = new Paint();
        cross.setColor(Color.rgb(255, 210, 90));
        cross.setStrokeWidth(6f);
        c.drawLine(w / 2f - 60, h / 2f, w / 2f + 60, h / 2f, cross);
        c.drawLine(w / 2f, h / 2f - 60, w / 2f, h / 2f + 60, cross);

        Paint text = new Paint();
        text.setColor(Color.WHITE);
        text.setTextSize(64f);
        text.setFakeBoldText(true);
        c.drawText("TOP-LEFT", 40, 90, text);
        c.drawText("TOP-RIGHT", w - 430, 90, text);
        c.drawText("BOT-LEFT", 40, h - 40, text);
        c.drawText("BOT-RIGHT", w - 440, h - 40, text);

        text.setTextSize(96f);
        text.setTextAlign(Paint.Align.CENTER);
        c.drawText("XrealBeam", w / 2f, h / 2f - 90, text);
        text.setTextSize(44f);
        text.setColor(Color.rgb(150, 200, 255));
        c.drawText("no video loaded - press 'video' on the phone", w / 2f, h / 2f + 160, text);

        return bmp;
    }

    /**
     * The FOV check overlay: a transparent bitmap carrying a frame at the extreme edge plus corner
     * brackets. If the wearer sees a gap between these brackets and the panel edge, the real FOV is
     * wider than the model; if the brackets are cut off, it is narrower.
     */
    private static Bitmap buildFovCheckFrame() {
        int w = 1920, h = 1080;
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        Canvas c = new Canvas(bmp);

        Paint frame = new Paint();
        frame.setColor(Color.argb(230, 255, 255, 0));
        frame.setStyle(Paint.Style.STROKE);
        frame.setStrokeWidth(4f);
        c.drawRect(2, 2, w - 2, h - 2, frame);

        Paint bracket = new Paint();
        bracket.setColor(Color.argb(255, 255, 60, 60));
        bracket.setStyle(Paint.Style.STROKE);
        bracket.setStrokeWidth(12f);
        int len = 120;
        int m = 12;
        c.drawLine(m, m, m + len, m, bracket);
        c.drawLine(m, m, m, m + len, bracket);
        c.drawLine(w - m, m, w - m - len, m, bracket);
        c.drawLine(w - m, m, w - m, m + len, bracket);
        c.drawLine(m, h - m, m + len, h - m, bracket);
        c.drawLine(m, h - m, m, h - m - len, bracket);
        c.drawLine(w - m, h - m, w - m - len, h - m, bracket);
        c.drawLine(w - m, h - m, w - m, h - m - len, bracket);

        Paint text = new Paint();
        text.setColor(Color.argb(255, 255, 255, 0));
        text.setTextSize(54f);
        text.setFakeBoldText(true);
        text.setTextAlign(Paint.Align.CENTER);
        c.drawText("FOV CHECK - brackets must sit ON the panel corners", w / 2f, 80f, text);
        c.drawText("margin visible = FOV bigger than assumed; cut off = smaller", w / 2f, h - 40f, text);

        Paint mid = new Paint();
        mid.setColor(Color.argb(200, 0, 255, 128));
        mid.setStrokeWidth(4f);
        c.drawLine(w / 2f, h / 2f - 40, w / 2f, h / 2f + 40, mid);
        c.drawLine(w / 2f - 40, h / 2f, w / 2f + 40, h / 2f, mid);

        return bmp;
    }

    private static int createTexture(Bitmap bmp) {
        int[] ids = new int[1];
        GLES20.glGenTextures(1, ids, 0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, ids[0]);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bmp, 0);
        bmp.recycle();
        return ids[0];
    }

    private static void perspective(float[] m, float fovyDeg, float aspect, float near, float far) {
        float f = (float) (1.0 / Math.tan(Math.toRadians(fovyDeg) / 2.0));
        for (int i = 0; i < 16; i++) m[i] = 0f;
        m[0] = f / aspect;
        m[5] = f;
        m[10] = (far + near) / (near - far);
        m[11] = -1f;
        m[14] = (2f * far * near) / (near - far);
    }

    /** Scale to the metric content size, then push out to -distance (column-major). */
    private static void scaleTranslate(float[] m, float sx, float sy, float z) {
        for (int i = 0; i < 16; i++) m[i] = 0f;
        m[0] = sx;
        m[5] = sy;
        m[10] = 1f;
        m[14] = z;
        m[15] = 1f;
    }

    /** Column-major 4x4 multiply: out = a * b. */
    private static void multiply(float[] out, float[] a, float[] b) {
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                float s = 0f;
                for (int k = 0; k < 4; k++) s += a[k * 4 + row] * b[col * 4 + k];
                out[col * 4 + row] = s;
            }
        }
    }
}
