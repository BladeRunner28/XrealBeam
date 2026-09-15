package com.xman.xrealbeam;

import android.app.Presentation;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.TypedValue;
import android.view.Display;
import android.view.Gravity;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.Locale;

/**
 * Full-screen presentation of the virtual screen on the glasses.
 *
 * Reality check on Samsung hardware: when DeX auto-starts, it claims the external display, and a
 * Presentation created against it may throw or end up redirected into the DeX desktop. The caller
 * catches that and falls back to an in-activity GL surface (which DeX shows as a window on the
 * glasses), so there is always something on screen to judge the pose path by.
 *
 * The HUD matters more than it looks: with the glasses on, the wearer cannot read the phone, so
 * every calibration prompt, the live stillness/tilt feedback and the pass criterion have to be here.
 */
public class GlassesPresentation extends Presentation {

    private final HeadPose pose;
    private final XrealImu imu;
    private final MountCal mountCal;
    private GLSurfaceView gl;
    private TextView hud;
    private TextView prompt;
    private final Handler h = new Handler(Looper.getMainLooper());

    private final Runnable hudTick = new Runnable() {
        @Override
        public void run() {
            if (hud != null) {
                hud.setText(String.format(Locale.US,
                        "%s | %.0f fps | %s | %s\naccel %6.2f %6.2f %6.2f g   |a| %.2f   spread %.3f g",
                        pose.isLeveled() ? "LEVELLED" : "not levelled",
                        pose.fps(), pose.describe(),
                        imu == null ? "no IMU" : imu.state,
                        imu == null ? 0 : imu.accelBodyX, imu == null ? 0 : imu.accelBodyY,
                        imu == null ? 0 : imu.accelBodyZ,
                        imu == null ? 0 : Math.sqrt(imu.accelBodyX * imu.accelBodyX
                                + imu.accelBodyY * imu.accelBodyY + imu.accelBodyZ * imu.accelBodyZ),
                        imu == null ? 0 : imu.spreadA));
            }
            if (prompt != null && mountCal != null) {
                String p = mountCal.phase() == MountCal.Phase.IDLE
                        ? "" : ("\n" + mountCal.prompt() + "\n     [" + mountCal.status + "]");
                prompt.setText(p);
            }
            h.postDelayed(this, 400);
        }
    };

    public GlassesPresentation(Context ctx, Display display, HeadPose pose, XrealImu imu,
                               MountCal mountCal) {
        super(ctx, display);
        this.pose = pose;
        this.imu = imu;
        this.mountCal = mountCal;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        FrameLayout root = new FrameLayout(getContext());

        gl = new GLSurfaceView(getContext());
        gl.setEGLContextClientVersion(2);
        gl.setRenderer(new ScreenRenderer(pose, imu));
        gl.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        root.addView(gl, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT));

        LinearLayout column = new LinearLayout(getContext());
        column.setOrientation(LinearLayout.VERTICAL);

        hud = new TextView(getContext());
        hud.setTextColor(0xCCFFFFFF);
        hud.setTextSize(TypedValue.COMPLEX_UNIT_SP, 14f);
        hud.setPadding(24, 24, 24, 0);
        column.addView(hud);

        // The calibration prompt: big enough to read at the edge of the field of view.
        prompt = new TextView(getContext());
        prompt.setTextColor(0xFFFFD24A);
        prompt.setTextSize(TypedValue.COMPLEX_UNIT_SP, 26f);
        prompt.setPadding(24, 8, 24, 24);
        column.addView(prompt);

        FrameLayout.LayoutParams lp = new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.WRAP_CONTENT, FrameLayout.LayoutParams.WRAP_CONTENT);
        lp.gravity = Gravity.TOP | Gravity.START;
        root.addView(column, lp);

        setContentView(root);
        h.post(hudTick);
    }

    @Override
    protected void onStart() {
        super.onStart();
        if (gl != null) gl.onResume();
    }

    @Override
    protected void onStop() {
        h.removeCallbacks(hudTick);
        if (gl != null) gl.onPause();
        super.onStop();
    }
}
