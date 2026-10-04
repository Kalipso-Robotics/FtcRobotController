package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Matrix;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

/**
 * Turns a detector's pixel boxes into balls on the field. All its numbers live in VisionConfig.
 *
 * Two ways to use it:
 *
 * 1. Piggyback on a detector (TFLite or colour blob). It registers as the detector's result
 *    listener, so it runs on the camera thread straight after detect() with that frame's
 *    capture time -- no executor, no polling:
 *
 *      TFLitePollenNectarDetector detector = new TFLitePollenNectarDetector(ctx);
 *      new Raytracer(detector, VisionConfig.ARDUCAM);
 *      new VisionManager.Builder(hw).withCamera(VisionConfig.ARDUCAM).addProcessor(detector).build();
 *
 *      List<BallInformation> balls = SharedData.getBallInformation(); // anywhere
 *
 *    A list is published every frame, empty included, so "nothing seen" is distinguishable
 *    from "stale": compare SharedData.getBallInformationTimeMs() to now. Position uses the pose
 *    at the frame's capture time (SharedData pose history), not the pose now.
 *
 * 2. Pull: new Raytracer(VisionConfig.ARDUCAM).fieldPos(recognition, growPx, robotPose).
 *
 * The camera mount is read from VisionConfig.ARDUCAM.mount every frame, so dashboard edits apply
 * live. A camera on a moving arm calls setMountSupplier with its forward kinematics instead.
 *
 * Maths: each box edge becomes an angle, the two angles are averaged per axis (exact: every
 * sensor column/row is a plane through the pinhole, and the tangent geometry inside it is a 2D
 * circle problem). The centre ray is rotated into the robot frame and intersected with the plane
 * at the ball's radius above the floor. The ball's angular radius also gives an independent
 * depth, reported on the Estimate as a consistency check. It is NOT fused in.
 */
public class Raytracer {

    private static final String TAG = "Raytracer";

    /**
     * OpenCV camera axes (right, down, forward) -> robot axes (left, up, forward).
     * A true rotation, not a mirror: det = +1, because both triads are right-handed.
     */
    static final Matrix CV_TO_ROBOT =
            new Matrix(new double[][]{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}});

    /** Result of one ball's raytrace. Immutable. Lengths in mm. */
    public static final class Estimate {
        /** Ball centre in the robot frame: x = left, y = forward. */
        public final Point robotPos;
        /** Camera-frame depth from the ray/plane intersection (the ray parameter t). */
        public final double rayDepthMM;
        /** Depth implied by the ball's vertical angular size (uses fy). */
        public final double sizeDepthVMM;
        /** Depth implied by the ball's horizontal angular size (uses fx). */
        public final double sizeDepthHMM;
        /** Centre ray angles: horizontal (psi) and vertical (theta), rad, from the optical axis. */
        public final double psiRad, thetaRad;
        /** Angular radius of the ball per axis, rad. */
        public final double deltaXRad, deltaYRad;
        /** The centre ray rotated into the robot frame (x left, y up, z forward), not normalised. */
        public final Vector3d rayRobot;

        Estimate(Point robotPos, double rayDepthMM, double sizeDepthVMM, double sizeDepthHMM,
                 double psiRad, double thetaRad, double deltaXRad, double deltaYRad, Vector3d rayRobot) {
            this.robotPos = robotPos;
            this.rayDepthMM = rayDepthMM;
            this.sizeDepthVMM = sizeDepthVMM;
            this.sizeDepthHMM = sizeDepthHMM;
            this.psiRad = psiRad;
            this.thetaRad = thetaRad;
            this.deltaXRad = deltaXRad;
            this.deltaYRad = deltaYRad;
            this.rayRobot = rayRobot;
        }

        /** (sizeDepthV - rayDepth) / rayDepth. Positive: the ball looks farther by size than by ray. */
        public double rangeDisagreement() {
            return (sizeDepthVMM - rayDepthMM) / rayDepthMM;
        }

        /** False for a ball off the floor, a merged/occluded blob or the wrong radius. */
        public boolean isConsistent() {
            return Math.abs(rangeDisagreement()) <= VisionConfig.MAX_RANGE_DISAGREEMENT;
        }

        /** Field position, given the robot pose at the moment the frame was captured. */
        public Point fieldPos(Position robotPoseAtCapture) {
            return toField(robotPos, robotPoseAtCapture);
        }

        @Override
        public String toString() {
            return String.format(Locale.US,
                    "Estimate{robot=(%.1f,%.1f) t=%.1f sizeV=%.1f sizeH=%.1f disagree=%+.1f%% ok=%b"
                            + " psi=%.2f theta=%.2f dX=%.2f dY=%.2f deg}",
                    robotPos.getX(), robotPos.getY(), rayDepthMM, sizeDepthVMM, sizeDepthHMM,
                    100 * rangeDisagreement(), isConsistent(),
                    Math.toDegrees(psiRad), Math.toDegrees(thetaRad),
                    Math.toDegrees(deltaXRad), Math.toDegrees(deltaYRad));
        }
    }

    /** One processed frame: the three lists are parallel (index i is the same ball). Immutable. */
    public static final class Result {
        public final long captureNanos;
        public final List<VisionRecognition> detections;
        public final List<Estimate> estimates;
        public final List<BallInformation> balls;

        Result(long captureNanos, List<VisionRecognition> detections,
               List<Estimate> estimates, List<BallInformation> balls) {
            this.captureNanos = captureNanos;
            this.detections = Collections.unmodifiableList(detections);
            this.estimates = Collections.unmodifiableList(estimates);
            this.balls = Collections.unmodifiableList(balls);
        }
    }

    private final VisionConfig.Camera cam;
    private final KVisionProcessor<?> detector; // null in pull mode
    private volatile Supplier<VisionConfig.Mount> mountSupplier;
    private volatile Result lastResult = new Result(0, new ArrayList<>(), new ArrayList<>(), new ArrayList<>());
    private boolean sizeChecked;

    /** Pull mode: no detector, no publishing. */
    public Raytracer(VisionConfig.Camera cam) {
        this.cam = cam;
        this.detector = null;
        this.mountSupplier = () -> cam.mount;
    }

    /** Piggyback mode: raytraces every frame the detector produces and publishes to SharedData. */
    public Raytracer(KVisionProcessor<List<VisionRecognition>> detector, VisionConfig.Camera cam) {
        this.cam = cam;
        this.detector = detector;
        this.mountSupplier = () -> cam.mount;
        detector.setResultListener(this::update);
    }

    /** For a camera on a moving arm: called once per frame, returns the mount at that moment. */
    public void setMountSupplier(Supplier<VisionConfig.Mount> supplier) { this.mountSupplier = supplier; }

    public VisionConfig.Camera getCamera() { return cam; }

    // -------------------------------------------------------------------------
    // Piggyback path
    // -------------------------------------------------------------------------

    /** Raytraces one frame and publishes it. Package-visible so tests can drive it directly. */
    void update(List<VisionRecognition> recs, long captureNanos) {
        checkFrameSizeOnce();
        Position pose = SharedData.getOdometryWheelIMUPositionAt(captureNanos);
        if (pose == null) {
            pose = SharedData.getOdometryWheelIMUPosition();
            KLog.d(TAG, () -> "No pose history at capture time, using latest pose");
        }

        List<BallInformation> balls = new ArrayList<>();
        List<Estimate> estimates = new ArrayList<>();
        List<VisionRecognition> used = new ArrayList<>();
        if (recs != null) {
            for (VisionRecognition rec : recs) {
                if (rec.confidence < VisionConfig.MIN_CONFIDENCE) continue;
                BallInformation.Type type = typeFor(rec.label);
                Estimate est = estimate(rec, diameterMM(type) / 2.0, VisionConfig.EDGE_GROW_PX);
                if (est == null) continue;

                balls.add(new BallInformation(est.fieldPos(pose), type, rec.confidence,
                        Math.hypot(est.robotPos.getX(), est.robotPos.getY())));
                estimates.add(est);
                used.add(rec);
            }
        }
        lastResult = new Result(captureNanos, used, estimates, balls);
        SharedData.setBallInformation(balls);
    }

    /** The last frame with its raw boxes and per-ball geometry (ray vs size depth), for diagnostics. */
    public Result getLastResult() { return lastResult; }

    private void checkFrameSizeOnce() {
        if (sizeChecked || detector == null) return;
        int w = detector.getFrameWidth(), h = detector.getFrameHeight();
        if (w == 0) return; // never init'd (tests)
        sizeChecked = true;
        if (w != cam.width || h != cam.height) {
            KLog.e(TAG, String.format(Locale.US,
                    "Stream is %dx%d but %s is calibrated for %dx%d. Every ball position will be wrong. "
                            + "Build the portal with VisionManager.Builder.withCamera(VisionConfig.ARDUCAM).",
                    w, h, cam.name, cam.width, cam.height));
        }
    }

    static BallInformation.Type typeFor(String label) {
        switch (label.toLowerCase(Locale.US)) {
            case "pollen":
            case "yellow":
                return BallInformation.Type.YELLOW;
            case "nectar_red":
            case "red":
                return BallInformation.Type.RED;
            case "nectar_blue":
            case "blue":
                return BallInformation.Type.BLUE;
            default:
                return BallInformation.Type.UNKNOWN;
        }
    }

    private static double diameterMM(BallInformation.Type type) {
        return type == BallInformation.Type.YELLOW
                ? VisionConfig.POLLEN_DIAMETER_MM
                : VisionConfig.NECTAR_DIAMETER_MM;
    }

    // -------------------------------------------------------------------------
    // Pull path
    // -------------------------------------------------------------------------

    /** Ball centre on the field, or null when the ball can't be raytraced. See estimate(). */
    public Point fieldPos(VisionRecognition det, double growPx, Position robotPose) {
        Estimate e = estimate(det, growPx);
        return e == null ? null : e.fieldPos(robotPose);
    }

    /** As estimate(det, radius, grow) with the detection's own diameter (colour blobs carry one). */
    public Estimate estimate(VisionRecognition det, double growPx) {
        return estimate(det, det.getRadiusMM(), growPx);
    }

    /**
     * Locates a ball from its box. growPx grows each edge outward first (0 for TFLite, whose boxes
     * are not eroded; VisionConfig.COLOR_BLOB_EDGE_GROW_PX for colour blobs).
     *
     * @return null when geometrically impossible: unknown radius, box clipped by the frame,
     *         flat/upward ray, the plane behind the camera, or farther than MAX_BALL_RANGE_MM.
     *         A non-null estimate may still be !isConsistent().
     */
    public Estimate estimate(VisionRecognition det, double radiusMM, double growPx) {
        if (radiusMM <= 0 || isClipped(det)) return null;
        return estimate(det.left - growPx, det.right + growPx, det.top - growPx, det.bottom + growPx,
                radiusMM, mountSupplier.get());
    }

    boolean isClipped(VisionRecognition det) {
        double m = VisionConfig.BORDER_MARGIN_PX;
        return det.left <= m || det.top <= m || det.right >= cam.width - m || det.bottom >= cam.height - m;
    }

    /** Edge-pixel form: no growth, no border gate, explicit mount. Used by the fit and the tests. */
    Estimate estimate(double uL, double uR, double vT, double vB, double radiusMM, VisionConfig.Mount mount) {
        double uc = (uL + uR) / 2.0, vc = (vT + vB) / 2.0;

        // edges -> angles
        double aL = angleX(uL, vc), aR = angleX(uR, vc);
        double aT = angleY(vT, uc), aB = angleY(vB, uc);

        // centre ray (mean angle) and angular radius delta (half the span), per axis
        double psi = (aL + aR) / 2.0, deltaX = (aR - aL) / 2.0;
        double theta = (aT + aB) / 2.0, deltaY = (aB - aT) / 2.0;

        // rotate (tan psi, tan theta, 1) into the robot frame
        Vector3d ray = rotate(camToRobot(mount), Math.tan(psi), Math.tan(theta), 1);
        Vector3d camPos = new Vector3d(mount.xMM, mount.yMM, mount.zMM);

        // intersect with the plane at the ball's radius; the ray's camera-frame z is exactly 1
        // and the rotation is rigid, so t is the ball's camera-frame depth
        double t = planeT(ray, camPos, radiusMM);
        if (Double.isNaN(t)) return null;
        Vector3d hit = camPos.add(ray.multiply(t));
        if (Math.hypot(hit.getX(), hit.getZ()) > VisionConfig.MAX_BALL_RANGE_MM) return null;

        return new Estimate(new Point(hit.getX(), hit.getZ()), t,
                sizeDepth(radiusMM, deltaY, theta), sizeDepth(radiusMM, deltaX, psi),
                psi, theta, deltaX, deltaY, ray);
    }

    // -------------------------------------------------------------------------
    // Geometry
    // -------------------------------------------------------------------------

    /**
     * Rotation mapping a normalised OpenCV ray ((u-cx)/fx, (v-cy)/fy, 1) into the robot frame.
     * Read right to left: flip to robot axes, roll about the optical axis, pitch the lens down,
     * then yaw it left.
     */
    static Matrix camToRobot(VisionConfig.Mount m) {
        return rotY(Math.toRadians(m.yawDeg))
                .multiply(rotX(Math.toRadians(m.pitchDeg)))
                .multiply(rotZ(Math.toRadians(m.rollDeg)))
                .multiply(CV_TO_ROBOT);
    }

    private static Matrix rotX(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Matrix(new double[][]{{1, 0, 0}, {0, c, -s}, {0, s, c}});
    }

    private static Matrix rotY(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Matrix(new double[][]{{c, 0, s}, {0, 1, 0}, {-s, 0, c}});
    }

    private static Matrix rotZ(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Matrix(new double[][]{{c, -s, 0}, {s, c, 0}, {0, 0, 1}});
    }

    private static Vector3d rotate(Matrix m, double x, double y, double z) {
        return new Vector3d(
                m.get(0, 0) * x + m.get(0, 1) * y + m.get(0, 2) * z,
                m.get(1, 0) * x + m.get(1, 1) * y + m.get(1, 2) * z,
                m.get(2, 0) * x + m.get(2, 1) * y + m.get(2, 2) * z);
    }

    /** Robot-frame point (x = left, y = forward) to field coordinates at robotPose. */
    static Point toField(Point local, Position robotPose) {
        if (local == null) return null;
        return new Position(local.getY(), local.getX(), 0).toNewFrame(robotPose).toPoint();
    }

    /**
     * Parameter t where camPos + t*dir meets the horizontal plane y = planeY (robot frame, y up),
     * or NaN for a flat/upward ray or a plane behind the camera. dir is not unit length, so
     * "flat" is judged against its own length: dir.y > -0.01 * |dir|.
     */
    private static double planeT(Vector3d dir, Vector3d camPos, double planeY) {
        double dirY = dir.getY();
        if (dirY > -0.01 * dir.magnitude()) return Double.NaN;
        double t = (planeY - camPos.getY()) / dirY;
        return t > 0 ? t : Double.NaN;
    }

    /** Depth implied by angular radius delta at centre angle c: r / sin(delta) * cos(c). */
    private static double sizeDepth(double radiusMM, double delta, double centreAngle) {
        return radiusMM / Math.sin(delta) * Math.cos(centreAngle);
    }

    /** Angle of pixel column u (row vRef only matters when undistorting). */
    private double angleX(double u, double vRef) {
        return Math.atan(VisionConfig.APPLY_DISTORTION ? undistort(u, vRef)[0] : (u - cam.cx) / cam.fx);
    }

    private double angleY(double v, double uRef) {
        return Math.atan(VisionConfig.APPLY_DISTORTION ? undistort(uRef, v)[1] : (v - cam.cy) / cam.fy);
    }

    // Distortion model, defined once: x_d = x * radial(r2) + tangential(x, y, r2).
    private double radial(double r2) {
        return 1 + cam.k1 * r2 + cam.k2 * r2 * r2 + cam.k3 * r2 * r2 * r2;
    }

    private double tangentialX(double x, double y, double r2) {
        return 2 * cam.p1 * x * y + cam.p2 * (r2 + 2 * x * x);
    }

    private double tangentialY(double x, double y, double r2) {
        return cam.p1 * (r2 + 2 * y * y) + 2 * cam.p2 * x * y;
    }

    /** Pixel -> undistorted normalised coordinates, same fixed-point scheme as cv::undistortPoints. */
    double[] undistort(double u, double v) {
        double x0 = (u - cam.cx) / cam.fx, y0 = (v - cam.cy) / cam.fy;
        double x = x0, y = y0;
        for (int i = 0; i < 5; i++) {
            double r2 = x * x + y * y;
            double rad = radial(r2);
            x = (x0 - tangentialX(x, y, r2)) / rad;
            y = (y0 - tangentialY(x, y, r2)) / rad;
        }
        return new double[]{x, y};
    }

    /** Test hook: the forward model undistort inverts. */
    double[] distort(double x, double y) {
        double r2 = x * x + y * y;
        double rad = radial(r2);
        return new double[]{x * rad + tangentialX(x, y, r2), y * rad + tangentialY(x, y, r2)};
    }
}
