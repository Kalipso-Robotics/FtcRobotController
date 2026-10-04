package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;

@Config
public class CameraIntrinsics{
    // Fixed: these set the VisionPortal resolution the intrinsics were scaled to.
    public static final int CAM_WIDTH  = 640;
    public static final int CAM_HEIGHT = 480;

    private final double fx, fy;
    private final double cx, cy;
    private final double mountAngle;
    private final CameraPose mount; // pitch-only pose, for the legacy floor/size paths
    private final double k1, k2, k3, p1, p2; //distortions

    /**
     * Apparent-size ranging is calibrated separately from the floor ray, because the
     * colour mask is a SMALLER disc than the ball's silhouette by a roughly constant
     * ~19 px at full resolution: the 5x5 Gaussian and 3x3 MORPH_OPEN run at 320x240 and
     * get doubled back to full res, and the HSV threshold clips the ball's shaded limb.
     *
     * The deficit is ADDITIVE, not multiplicative -- fitting
     *     mean(bboxW, bboxH) = SIZE_FOCAL_PX * D / range + BLOB_EDGE_DEFICIT_PX
     * to RaytracingGroundTruth_2026_09_08 gives R^2=0.997 over 508-1422mm, and range
     * error mean +3.9mm / sd 24.5mm, inside max(5%, 50mm) at all ten distances. That is
     * why the old size ranging fell apart with distance: 19 px is 11% of a 183 px blob
     * at 508mm but 31% of a 47 px blob at 1422mm.
     *
     * This is NOT a lens intrinsic and must not be read as one. It is a lumped empirical
     * constant for "blob pixels per unit angular size of THIS ball under THIS
     * segmentation", which is why it is named SIZE_FOCAL_PX and not fx.
     *
     * ponytail: it sits 18% above fy (629.8 vs 532.3) and physically should not. Part of
     * that gap is the segmentation, part is that distortion is never applied, and part
     * may be the assumed 127mm diameter -- a true focal of 532 would imply the visible
     * blob is ~150mm across. Validating it against the same CSV it was fitted from is
     * circular, so treat it as a calibration knob, not a measurement. Real fix: a
     * checkerboard calibration at 640x480 directly rather than rescaled from 1280x800
     * (see calibrate_camera.py), plus a caliper on the artifact.
     *
     * Note also that bbox dimensions are quantised to EVEN full-res pixels (320x240
     * processing, x2 in scaleRectToFullResolution), so +/-1 processing pixel is ~4% of a
     * 47 px blob at 1422mm. That is the noise floor here; no constant removes it.
     */
    public static double SIZE_FOCAL_PX        = 629.786;
    public static double BLOB_EDGE_DEFICIT_PX = -19.290;
    // fx, cx and the distortion coeffs still come from the 1280x800 checkerboard
    // (see calibrate_camera.py) rescaled by 0.5 on x. That rescale is KNOWN WRONG --
    // real balls read a bbox aspect of 1.01-1.04 in the clean 610-813mm band while
    // fx/fy says 0.834, so the pixels are square-ish and the camera crops rather than
    // squashing. fx/cx are left as-is only because they are not yet solvable: every row
    // of RaytracingGroundTruth_2026_09_08 is centred (KnownLateralMM 0), so a horizontal
    // fit has no lever arm but the camera's own 157.5mm offset and just re-measures it.
    // Collect ~6 off-centre bursts and re-run fit_intrinsics.py to finish the job.
    //
    // DO NOT "fit" fx/fy/cx/cy from ground-truth distance CSVs. They are properties of
    // the lens and sensor; the only honest way to change them is a checkerboard
    // calibration (calibrate_camera.py). Solving them from tape-measured ball distances
    // just launders errors in cam height, cam offset, mount angle and the blob's
    // floor-contact assumption INTO the lens model, where they are invisible. That was
    // tried on the 2026-09-08 CSV and it fitted fy=555.5/cy=171.5 -- a principal point
    // 68px off centre in a 480-tall image, which no real sensor has. It also scored
    // WORSE end to end (RMS 28.7mm, worst 67.4mm) than leaving the intrinsics alone.
    //
    // MOUNT ANGLE is the exception and the one number that was fitted here. It is a
    // property of the bracket, not the lens, it is a single parameter, and with the
    // intrinsics held fixed it is sharply identifiable: RMS over the ten ground-truth
    // distances is 570mm at 24 deg, 69.6mm at 28, 24.6mm at 29, 74.2mm at 30. 29 deg it
    // is, and with it the floor projection lands inside max(5%, 50mm) at every distance.
    //
    // The bracket is nominally 24 deg. The data says 29 and leaves no room to argue: no
    // plausible mounting geometry rescues 24 (the best-fitting cam height for it is
    // 150mm against a measured 236, still at RMS 132). Re-measure the assembled tilt --
    // this constant is currently absorbing whatever that discrepancy really is.
    public static final CameraIntrinsics ARDUCAM = new CameraIntrinsics(
            444.14195, 532.27560,
            350.01860, 212.43984,
            0.045011, -0.059862, 0.000330,
            0.001499, 0.005590,
            CameraPose.ARDUCAM_PITCH_RAD,
            CameraPose.ARDUCAM_OFFSET // z=151.868 before tilt
    );


    /** Pixel (u, v) as an un-normalised ray direction in the robot frame, via the legacy mount. */
    private Vector3d rayInRobotFrame(double u, double v) {
        return mount.toRobot((u - cx) / fx, (v - cy) / fy, 1);
    }

    /** Robot-frame point (x = left, y = forward) to field coordinates at robotPose. */
    static Point toField(Point local, Position robotPose) {
        if (local == null) return null;
        return new Position(local.getY(), local.getX(), 0).toNewFrame(robotPose).toPoint();
    }

    public CameraIntrinsics(double fx, double fy, double cx, double cy,
                            double k1, double k2, double k3, double p1, double p2,
                            double mountAngle, Vector3d cameraOffset) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.k1 = k1;
        this.k2 = k2;
        this.k3 = k3;
        this.p1 = p1;
        this.p2 = p2;
        this.mountAngle = mountAngle;
        this.mount = CameraPose.fromAngles(mountAngle, 0, 0, cameraOffset);
    }

    public CameraIntrinsics(double fx, double fy, double cx, double cy,
                            double mountAngle, Vector3d cameraOffset) {
        this(fx, fy, cx, cy, 0, 0, 0, 0, 0, mountAngle, cameraOffset);
    }

    /** @deprecated legacy floor-ray path; build a CameraPose and use estimateBall. */
    @Deprecated
    public CameraIntrinsics withMount(double mountAngle, Vector3d offset) {
        return new CameraIntrinsics(fx, fy, cx, cy, k1, k2, k3, p1, p2,
                mountAngle, offset);
    }

    /** @deprecated legacy floor-ray path; migrate to estimateBall + BallEstimate.fieldPos. */
    @Deprecated
    public Point calculateWorldPos(double pixelX, double pixelY, Position robotPose) {
        return toField(calculateRobotFramePos(pixelX, pixelY), robotPose);
    }

    /**
     * LEGACY: bottom-edge pixel intersected with the floor, via the pitch-only mount rotation.
     * Its 29 deg pitch absorbs the colour mask's bottom-edge bias. Migrate to estimateBall
     * once the pitch has been refit for that method.
     */
    @Deprecated
    public Point calculateRobotFramePos(double pixelX, double pixelY) {
        Vector3d dir = rayInRobotFrame(pixelX, pixelY);
        double t = planeT(dir, mount.position, 0);
        if (Double.isNaN(t)) return null;
        Vector3d hit = mount.position.add(dir.multiply(t));
        return new Point(hit.getX(), hit.getZ());
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

    /** Edges are grown outward by half the colour mask's constant shrink before use. */
    public static double edgeGrowPx() { return -BLOB_EDGE_DEFICIT_PX / 2.0; }

    /** A bbox this close to the frame edge is probably clipped, so its centre and size are wrong. */
    public static double BORDER_MARGIN_PX = 2;

    /** |size depth - ray depth| / ray depth above which a detection is flagged inconsistent. */
    public static double MAX_RANGE_DISAGREEMENT = 0.25;

    /** Farthest a ball can be from the robot: the field diagonal. Beyond it the point is rejected. */
    public static double MAX_BALL_RANGE_MM = 3657.6 * Math.sqrt(2);

    /**
     * Off by default: the distortion coefficients were fitted at 1280x800 and fx is known
     * ~16% low at 640x480, so undistorting would be built on a known-bad base. Effect is
     * only ~2.5% radial at the image corner.
     */
    public static boolean APPLY_DISTORTION = false;

    /**
     * Locates a ball from its silhouette edges. Each edge is converted to an angle first and
     * the two angles are averaged per axis -- exact, because every sensor column/row is a
     * plane through the pinhole and the tangent geometry inside it is a 2D circle problem.
     * The centre ray is then intersected with the plane at the ball's own radius above the
     * floor, using whatever camera pose is passed in (fixed mount or arm kinematics).
     *
     * The ball's angular radius also gives an independent depth (r / sin(delta) * cos(centre
     * angle)), reported on the estimate as a consistency check. It is NOT fused in.
     *
     * @return null when geometrically impossible: unknown radius, bbox clipped by the frame,
     *         flat/upward ray, the plane behind the camera, or farther than MAX_BALL_RANGE_MM.
     *         A non-null estimate may still be !isConsistent().
     */
    public BallEstimate estimateBall(VisionRecognition det, CameraPose cam) {
        double r = det.getRadiusMM();
        if (r <= 0 || isClipped(det)) return null;
        double g = edgeGrowPx();
        return estimateBall(det.left - g, det.right + g, det.top - g, det.bottom + g, r, cam);
    }

    private static boolean isClipped(VisionRecognition det) {
        return det.left <= BORDER_MARGIN_PX || det.top <= BORDER_MARGIN_PX
                || det.right >= CAM_WIDTH - BORDER_MARGIN_PX
                || det.bottom >= CAM_HEIGHT - BORDER_MARGIN_PX;
    }

    /** Edge-pixel form of estimateBall: no growth, no border gate. Used directly by tests. */
    BallEstimate estimateBall(double uL, double uR, double vT, double vB, double radiusMM, CameraPose cam) {
        double uc = (uL + uR) / 2.0, vc = (vT + vB) / 2.0;

        // edges -> angles
        double aL = angleX(uL, vc), aR = angleX(uR, vc);
        double aT = angleY(vT, uc), aB = angleY(vB, uc);

        // centre ray (mean angle) and angular radius delta (half the span), per axis
        double psi = (aL + aR) / 2.0, deltaX = (aR - aL) / 2.0;
        double theta = (aT + aB) / 2.0, deltaY = (aB - aT) / 2.0;

        // rotate (tan psi, tan theta, 1) into the robot frame
        Vector3d ray = cam.toRobot(Math.tan(psi), Math.tan(theta), 1);

        // intersect with the plane at the ball's radius; the ray's camera-frame z is exactly 1
        // and the rotation is rigid, so t is the ball's camera-frame depth
        double t = planeT(ray, cam.position, radiusMM);
        if (Double.isNaN(t)) return null;
        Vector3d hit = cam.position.add(ray.multiply(t));
        if (Math.hypot(hit.getX(), hit.getZ()) > MAX_BALL_RANGE_MM) return null;

        // size cross-check, per axis: reported, not fused
        double sizeV = sizeDepth(radiusMM, deltaY, theta);
        double sizeH = sizeDepth(radiusMM, deltaX, psi);
        return new BallEstimate(new Point(hit.getX(), hit.getZ()), t, sizeV, sizeH,
                psi, theta, deltaX, deltaY, ray, MAX_RANGE_DISAGREEMENT);
    }

    /** Depth implied by angular radius delta at centre angle c: r / sin(delta) * cos(c). */
    private static double sizeDepth(double radiusMM, double delta, double centreAngle) {
        return radiusMM / Math.sin(delta) * Math.cos(centreAngle);
    }

    /** Angle of pixel column u (row vRef only matters when undistorting). */
    private double angleX(double u, double vRef) {
        return Math.atan(APPLY_DISTORTION ? undistort(u, vRef)[0] : (u - cx) / fx);
    }

    private double angleY(double v, double uRef) {
        return Math.atan(APPLY_DISTORTION ? undistort(uRef, v)[1] : (v - cy) / fy);
    }

    // Distortion model, defined once: x_d = x * radial(r2) + tangential(x, y, r2).
    private double radial(double r2) {
        return 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    }

    private double tangentialX(double x, double y, double r2) {
        return 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    }

    private double tangentialY(double x, double y, double r2) {
        return p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    }

    /** Pixel -> undistorted normalised coordinates, same fixed-point scheme as cv::undistortPoints. */
    double[] undistort(double u, double v) {
        double x0 = (u - cx) / fx, y0 = (v - cy) / fy;
        double x = x0, y = y0;
        for (int i = 0; i < 5; i++) {
            double r2 = x * x + y * y;
            double rad = radial(r2);
            double dx = tangentialX(x, y, r2), dy = tangentialY(x, y, r2);
            x = (x0 - dx) / rad;
            y = (y0 - dy) / rad;
        }
        return new double[]{x, y};
    }

    /** Test hook: the forward model undistort inverts. */
    double[] distort(double x, double y) {
        double r2 = x * x + y * y;
        double rad = radial(r2);
        return new double[]{x * rad + tangentialX(x, y, r2), y * rad + tangentialY(x, y, r2)};
    }

    @Deprecated
    public double getDistanceFromRobot(double pixelX, double pixelY, Position robotPose) {
        Point object = calculateWorldPos(pixelX, pixelY, robotPose);
        if (object == null) {
            return Double.POSITIVE_INFINITY;
        }
        return robotPose.toPoint().distanceTo(object);
    }

    @Deprecated
    public double getDistanceFromRobot(VisionRecognition recognition, Position robotPose) {
        Point bottomCenter = recognition.getBottomMiddlePixel();
        return getDistanceFromRobot(bottomCenter.getX(), bottomCenter.getY(), robotPose);
    }

    /**
     * Ranges by known real-world object size instead of floor-plane intersection.
     * Unlike calculateRobotFramePos, this works for objects not resting on the floor
     * (elevated, stacked, mid-air) since it never assumes a floor plane.
     *
     * @param objectDiameterMM real-world diameter of the (assumed circular) object,
     *                         e.g. KColorBlobProcessor.getObjectDiameterMM()
     */
    @Deprecated
    public Point calculateRobotFramePosFromSize(VisionRecognition recognition, double objectDiameterMM) {
        double pixelWidth  = recognition.getWidth();
        double pixelHeight = recognition.getHeight();
        if (pixelWidth <= 0 || pixelHeight <= 0) return null;

        // The blob is a shrunken disc, not the ball's silhouette: subtract the measured
        // edge deficit before ranging, and use the focal that was fitted WITH that
        // deficit (SIZE_FOCAL_PX), not fx/fy -- see the constants' javadoc. Averaging
        // the two pixel dimensions is what the fit is calibrated against, and it is also
        // the right thing for a sphere, whose projection is the same angular size on
        // both axes.
        double apparentDiameterPx =
                (pixelWidth + pixelHeight) / 2.0 - BLOB_EDGE_DEFICIT_PX;
        if (apparentDiameterPx <= 0) return null;
        double range = (objectDiameterMM * SIZE_FOCAL_PX) / apparentDiameterPx;

        // NORMALISE before scaling: `range` is a true camera-to-object distance (that is
        // what SIZE_FOCAL_PX was fitted against), but the ray has length sqrt(x^2+y^2+1),
        // up to 1.14 at the image edge, so scaling it un-normalised overshoots by ~5-14%.
        // The floor/ball paths don't need this -- a ray-plane intersection is
        // scale-invariant in the direction vector.
        Vector3d dir = rayInRobotFrame(recognition.center.getX(), recognition.center.getY());
        Vector3d hit = mount.position.add(dir.normalize().multiply(range));
        return new Point(hit.getX(), hit.getZ());
    }

    @Deprecated
    public Point calculateWorldPosFromSize(VisionRecognition recognition, double objectDiameterMM, Position robotPose) {
        return toField(calculateRobotFramePosFromSize(recognition, objectDiameterMM), robotPose);
    }

    @Deprecated
    public double getDistanceFromRobotBySize(VisionRecognition recognition, double objectDiameterMM, Position robotPose) {
        Point object = calculateWorldPosFromSize(recognition, objectDiameterMM, robotPose);
        if (object == null) {
            return Double.POSITIVE_INFINITY;
        }
        return robotPose.toPoint().distanceTo(object);
    }

    public double getCx() { return cx; }
    public double getCy() { return cy; }
    public double getFx() { return fx; }
    public double getFy() { return fy; }
    public double getMountAngle() { return mountAngle; }
    public Vector3d getCameraOffset() { return mount.position; }

    /**
     * Expected bounding-box width/height ratio for a sphere, = fx/fy.
     *
     * A sphere subtending angle a projects to fx*a pixels wide by fy*a tall, so
     * this is NOT 1.0 unless fx == fy.
     *
     * That check has now been run and it FAILED: real balls read 1.01-1.04 in the clean
     * 610-813mm band of RaytracingGroundTruth_2026_09_08, against the 0.834 the shipped
     * fx/fy predicts. The pixels are square-ish and the 0.5/0.6 anamorphic rescale is
     * wrong. fy has since been re-fitted, but fx has not (no off-centre samples exist to
     * solve it), so this ratio is still built on a known-bad fx and currently reads 0.80.
     *
     * Consequence for callers: the aspect gate in KColorBlobProcessor is comparing
     * against a value ~20% below what real spheres produce, so its tolerance is doing
     * the work. Re-review that tolerance once fx is solved -- do not tighten it before.
     * Outside that band the measurement drifts anyway (1.18 at 508mm, 1.30 at 1422mm) as
     * the edge deficit eats height faster than width.
     */
    public double getExpectedSphereAspect() { return fx / fy; }
}
