package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.PollenColorBlobDetectionProcessor;

/**
 * Every number the camera and the Raytracer need, in one place (like OdometryConfig).
 *
 * A Camera is a lens (final: only a checkerboard run may change it) plus a Mount (live on the
 * dashboard: edit pitch/yaw/roll/offset and the next frame uses it). Everything that needs the
 * Arducam -- Raytracer, AprilTag, VisionManager -- takes VisionConfig.ARDUCAM, so there is no
 * pairing of resolution, lens and mount to get wrong.
 */
@Config
public class VisionConfig {

    /**
     * Where the lens sits in the robot frame and how it is tilted. Mutable so the dashboard (or
     * arm kinematics, via Raytracer.setMountSupplier) can change it between frames.
     * Offsets: x left (negative = right of centre), y up, z forward, mm. Angles in degrees.
     */
    public static class Mount {
        public double pitchDeg, yawDeg, rollDeg;
        public double xMM, yMM, zMM;

        public Mount(double pitchDeg, double yawDeg, double rollDeg, double xMM, double yMM, double zMM) {
            this.pitchDeg = pitchDeg;
            this.yawDeg = yawDeg;
            this.rollDeg = rollDeg;
            this.xMM = xMM;
            this.yMM = yMM;
            this.zMM = zMM;
        }
    }

    /** A webcam: hardware name, the stream size its lens calibration is valid for, lens, mount. */
    public static class Camera {
        public final String name;
        public final int width, height;
        public final double fx, fy, cx, cy;
        public final double k1, k2, k3, p1, p2; // distortion
        public Mount mount;

        public Camera(String name, int width, int height, double fx, double fy, double cx, double cy,
                      double k1, double k2, double k3, double p1, double p2, Mount mount) {
            this.name = name;
            this.width = width;
            this.height = height;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            this.k1 = k1;
            this.k2 = k2;
            this.k3 = k3;
            this.p1 = p1;
            this.p2 = p2;
            this.mount = mount;
        }
    }

    /**
     * The Arducam OV9782 at 1280x720 (the stream the TFLite detector and the colour blobs run on).
     *
     * PROVISIONAL lens: derived, not measured at this size. The OV9782's native mode is 1280x800
     * and 1280x720 is almost certainly a 40 px top/bottom centre crop, which leaves fx, fy, cx
     * and distortion untouched and moves only cy (354.066 - 40). Replace with a checkerboard
     * run at 1280x720 (calibrate_camera.py).
     *
     * ponytail: mount pitch 28 deg was fitted on 2026-09-08 ground truth at 640x480 (14mm RMS
     * forward). It is not the nominal 24 deg bracket: it soaks up whatever else is off. Refit it
     * from 1280x720 ground truth with RaytracerFitTest, which also solves yaw and roll.
     * ponytail: a Limelight solves pose on-device. Add a Camera for it here only if it ever needs
     * our ray.
     */
    public static Camera ARDUCAM = new Camera("Arducam", 1280, 720,
            888.2839, 887.1260, 700.0372, 314.0664,
            0.045011, -0.059862, 0.000330, 0.001499, 0.005590,
            new Mount(28, 0, 0, -157.548, 236.163, 163.470));

    // ---- ball detection ----

    /** Detections below this are not raytraced. */
    public static double MIN_CONFIDENCE = 0.35;

    public static double POLLEN_DIAMETER_MM = PollenColorBlobDetectionProcessor.POLLEN_DIAMETER_MM;
    // ponytail: assumed equal to pollen. BallCluster assumes 71 mm; put calipers on a nectar ball.
    public static double NECTAR_DIAMETER_MM = 2.8 * 25.4;

    /** Pixels to grow each box edge before raytracing. 0 for TFLite, whose boxes are not eroded. */
    public static double EDGE_GROW_PX = 0;
    /**
     * Same, for colour-blob masks, which are a smaller disc than the ball by a roughly constant
     * 19 px at 640x480 (the Gaussian + MORPH_OPEN erosion and the HSV clip of the shaded limb).
     * Half of that per edge, doubled for 1280x720.
     * ponytail: scaled from the 640 measurement, never re-measured at 1280.
     */
    public static double COLOR_BLOB_EDGE_GROW_PX = 19.29;

    // ---- raytracer gates ----

    /** A box this close to the frame edge is probably clipped, so its centre and size are wrong. */
    public static double BORDER_MARGIN_PX = 2;
    /** |size depth - ray depth| / ray depth above which a detection is flagged inconsistent. */
    public static double MAX_RANGE_DISAGREEMENT = 0.25;
    /** Farthest a ball can be from the robot: the field diagonal. Beyond it the point is rejected. */
    public static double MAX_BALL_RANGE_MM = 3657.6 * Math.sqrt(2);
    /**
     * Off by default. The coefficients were fitted at 1280x800, so they are native here, but
     * the effect is only ~2.5% radial at the corner and has not been validated on 1280x720.
     */
    public static boolean APPLY_DISTORTION = false;
}
