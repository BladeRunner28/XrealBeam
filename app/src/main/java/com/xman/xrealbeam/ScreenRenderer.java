package com.xman.xrealbeam;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.opengl.GLSurfaceView;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Renders the virtual screen: one textured quad at 2 m, viewed through the head pose.
 *
 * The content is deliberately procedural (grid + corner markers + crosshair) rather than a mirrored
 * phone screen. Reason: the only thing that cannot be verified off-device is whether head motion
 * moves the screen the right way, and a high-contrast grid makes a wrong axis or inverted sign
 * unmistakable. Swapping in a MediaProjection / SurfaceTexture content source is a later step and
 * touches only the texture, not the pose path.
 */
public class ScreenRenderer implements GLSurfaceView.Renderer {

    private static final String VS =
            "uniform mat4 uMvp;\n" +
            "attribute vec4 aPos;\n" +
            "attribute vec2 aUv;\n" +
            "varying vec2 vUv;\n" +
            "void main() {\n" +
            "  vUv = aUv;\n" +
            "  gl_Position = uMvp * aPos;\n" +
            "}\n";

    private static final String FS =
            "precision mediump float;\n" +
            "uniform sampler2D uTex;\n" +
            "varying vec2 vUv;\n" +
            "void main() {\n" +
            "  gl_FragColor = texture2D(uTex, vUv);\n" +
            "}\n";

    // Quad: 1.8 m wide, 1.012 m tall (16:9), centred on the origin, facing +z.
    private static final float[] QUAD = {
            -0.9f, -0.506f, 0f,
             0.9f, -0.506f, 0f,
            -0.9f,  0.506f, 0f,
             0.9f,  0.506f, 0f,
    };
    private static final float[] UV = {0f, 1f, 1f, 1f, 0f, 0f, 1f, 0f};

    private final HeadPose pose;
    private final XrealImu imu;

    private int program, aPos, aUv, uMvp, uTex;
    private int textureId;
    private final float[] proj = new float[16];
    private final float[] view = new float[16];
    private final float[] model = new float[16];
    private final float[] mvp = new float[16];
    private final float[] scratch = new float[16];
    private long lastFrameNs;
    private int width = 1920, height = 1080;

    public ScreenRenderer(HeadPose pose, XrealImu imu) {
        this.pose = pose;
        this.imu = imu;
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        program = build();
        aPos = GLES20.glGetAttribLocation(program, "aPos");
        aUv = GLES20.glGetAttribLocation(program, "aUv");
        uMvp = GLES20.glGetUniformLocation(program, "uMvp");
        uTex = GLES20.glGetUniformLocation(program, "uTex");

        textureId = createTexture(buildTestPattern());
        GLES20.glClearColor(0.02f, 0.02f, 0.04f, 1f);
        GLES20.glDisable(GLES20.GL_DEPTH_TEST);
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int w, int h) {
        width = Math.max(1, w);
        height = Math.max(1, h);
        GLES20.glViewport(0, 0, width, height);
        perspective(proj, 55.0f, (float) width / (float) height, 0.1f, 100f);
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        // Pose from the latest IMU quaternion, with the real frame interval for the follow filter.
        long now = System.nanoTime();
        double dt = lastFrameNs == 0 ? 1.0 / 60.0 : (now - lastFrameNs) / 1e9;
        lastFrameNs = now;
        pose.update(imu.quatW, imu.quatX, imu.quatY, imu.quatZ, dt);
        pose.getViewMatrix(view);
        translate(model, 0f, 0f, -2.0f);
        multiply(scratch, view, model);
        multiply(mvp, proj, scratch);

        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);
        GLES20.glUseProgram(program);

        FloatBuffer vb = buf(QUAD);
        FloatBuffer tb = buf(UV);
        GLES20.glEnableVertexAttribArray(aPos);
        GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, vb);
        GLES20.glEnableVertexAttribArray(aUv);
        GLES20.glVertexAttribPointer(aUv, 2, GLES20.GL_FLOAT, false, 0, tb);

        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textureId);
        GLES20.glUniform1i(uTex, 0);
        GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0);
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4);

        GLES20.glDisableVertexAttribArray(aPos);
        GLES20.glDisableVertexAttribArray(aUv);
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

    private static int build() {
        int vs = compile(GLES20.GL_VERTEX_SHADER, VS);
        int fs = compile(GLES20.GL_FRAGMENT_SHADER, FS);
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
        c.drawText("turn your head — this quad should stay put in space", w / 2f, h / 2f + 160, text);

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

    private static void translate(float[] m, float x, float y, float z) {
        for (int i = 0; i < 16; i++) m[i] = 0f;
        m[0] = 1; m[5] = 1; m[10] = 1; m[15] = 1;
        m[12] = x; m[13] = y; m[14] = z;
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
