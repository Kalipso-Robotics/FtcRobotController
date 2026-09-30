package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Matrix;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;

public class CameraIntrinsics{
    public static int  CAM_WIDTH   = 640;
    public static int  CAM_HEIGHT  = 480;

    private final double fx, fy;
    private final double cx, cy;
    private final double mountAngle;
    private final Matrix mountRotation; // cameraToRobot(mountAngle, 0, 0), for the floor/size paths
    private final Vector3d cameraOffset;
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
            Math.toRadians(29),
            new Vector3d(-157.548, 236.163, 163.470) // offsets, z=151.868 before tilt,
    );


    /**
     * Builds a row-major 3x3 rotation that maps a normalised OpenCV camera ray
     * ((u-cx)/fx, (v-cy)/fy, 1) -- x right, y down, z forward out of the lens --
     * into the robot frame (x left, y up, z forward), for use with
     * calculateBallRobotFramePos.
     *
     * R = Ry(yaw) . Rx(pitchDown) . Rz(roll) . diag(-1,-1,1)
     *
     * diag(-1,-1,1) is the same right-to-left / down-to-up flip the old floor-ray
     * code does inline via (cx-u), (cy-v). Rx(pitchDown) alone reproduces the old
     * pitch-only transform exactly: world_y = y*cos(theta) - z*sin(theta),
     * world_z = y*sin(theta) + z*cos(theta). Positive pitchDown tilts the lens
     * down (matches the old mountAngle sign), positive yaw turns the lens left,
     * and roll is about the optical axis. All three are in radians.
     */
    public static Matrix cameraToRobot(double pitchDown, double yaw, double roll) {
        double cp = Math.cos(pitchDown), sp = Math.sin(pitchDown);
        double cy2 = Math.cos(yaw), sy2 = Math.sin(yaw);
        double cr = Math.cos(roll), sr = Math.sin(roll);

        Matrix rx = new Matrix(new double[][]{{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}});
        Matrix ry = new Matrix(new double[][]{{cy2, 0, sy2}, {0, 1, 0}, {-sy2, 0, cy2}});
        Matrix rz = new Matrix(new double[][]{{cr, -sr, 0}, {sr, cr, 0}, {0, 0, 1}});
        Matrix flip = new Matrix(new double[][]{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}});

        return ry.multiply(rx).multiply(rz).multiply(flip);
    }

    /** Pixel (u, v) as an un-normalised ray direction in the robot frame, rotated by camToRobot. */
    private Vector3d rayInRobotFrame(double u, double v, Matrix camToRobot) {
        Matrix ray = new Matrix(new double[][]{{(u - cx) / fx}, {(v - cy) / fy}, {1}});
        Matrix dir = camToRobot.multiply(ray);
        return new Vector3d(dir.get(0, 0), dir.get(1, 0), dir.get(2, 0));
    }

    /** Robot-frame point (x = left, y = forward) to field coordinates at robotPose. */
    private static Point toField(Point local, Position robotPose) {
        if (local == null) return null;
        return new Position(local.getY(), local.getX(), 0).toNewFrame(robotPose).toPoint();
    }

    /**
     * Pitch-only mount rotation for the ball-ray path (calculateBallRobotFramePos /
     * calculateBallWorldPos). Starts at the old floor-ray mountAngle (29 deg) but is a
     * SEPARATE constant on purpose: that 29 deg was fitted against the bottom-edge pixel,
     * which is biased by the colour mask's shrink, so it does not carry over to a
     * centre-pixel ray. MUST BE REFIT against ground-truth data using the centre-ray /
     * ball-radius-plane model (see fit_intrinsics.py --ball-pitch-sweep) before this is
     * trusted for anything beyond the unit test's hand-derived case.
     */
    public static final Matrix ARDUCAM_BALL_MOUNT =
            cameraToRobot(Math.toRadians(29), 0, 0);

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
        this.mountRotation = cameraToRobot(mountAngle, 0, 0);
        this.cameraOffset = cameraOffset;
    }

    public CameraIntrinsics(double fx, double fy, double cx, double cy,
                            double mountAngle, Vector3d cameraOffset) {
        this(fx, fy, cx, cy, 0, 0, 0, 0, 0, mountAngle, cameraOffset);
    }

    public CameraIntrinsics withMount(double mountAngle, Vector3d offset) {
        return new CameraIntrinsics(fx, fy, cx, cy, k1, k2, k3, p1, p2,
                mountAngle, offset);
    }

    public Point calculateWorldPos(double pixelX, double pixelY, Position robotPose) {
        return toField(calculateRobotFramePos(pixelX, pixelY), robotPose);
    }

    /** Bottom-edge pixel intersected with the floor, via the pitch-only mount rotation. */
    public Point calculateRobotFramePos(double pixelX, double pixelY) {
        return calculateBallRobotFramePos(pixelX, pixelY, 0, mountRotation, cameraOffset);
    }

    public double getDistanceFromRobot(double pixelX, double pixelY, Position robotPose) {
        Point object = calculateWorldPos(pixelX, pixelY, robotPose);
        if (object == null) {
            return Double.POSITIVE_INFINITY;
        }
        return robotPose.toPoint().distanceTo(object);
    }

    public double getDistanceFromRobot(VisionRecognition recognition, Position robotPose) {
        Point bottomCenter = recognition.getBottomMiddlePixel();
        return getDistanceFromRobot(bottomCenter.getX(), bottomCenter.getY(), robotPose);
    }

    /**
     * Ray-plane intersection through the BLOB CENTRE pixel, hitting the horizontal plane
     * at the ball's own radius above the floor -- not the floor itself. The colour mask
     * shrinks the blob evenly on all sides, so the centre pixel stays unbiased where the
     * bottom edge (used by calculateRobotFramePos) is not.
     *
     * The ray direction is built from an explicit camToRobot rotation (see
     * cameraToRobot), so a mount with yaw/roll -- or one that moves at runtime -- can
     * supply its own rotation. calculateRobotFramePos is this with radius 0 and the
     * pitch-only mount rotation. The direction is never normalised: t is solved directly against
     * its un-normalised y-component, which is exact for a plane intersection.
     *
     * @param u          pixel X of the blob centre
     * @param v          pixel Y of the blob centre
     * @param radiusMM   ball radius above the floor, e.g. VisionRecognition.getRadiusMM().
     *                   0 is legal and means the floor plane itself.
     * @param camToRobot rotation from cameraToRobot(), camera ray -> robot frame
     * @param camPos     camera position in the robot frame, e.g. ARDUCAM.getCameraOffset()
     */
    public Point calculateBallRobotFramePos(double u, double v, double radiusMM,
                                             Matrix camToRobot, Vector3d camPos) {
        Vector3d dir = rayInRobotFrame(u, v, camToRobot);
        double dirY = dir.getY();
        if (dirY > -0.01) {
            return null;
        }

        double t = (radiusMM - camPos.getY()) / dirY;
        if (t <= 0) {
            return null;
        }

        Vector3d hit = camPos.add(dir.multiply(t));
        return new Point(hit.getX(), hit.getZ());
    }

    /**
     * Ball-ray overload keyed off a detection's own centre pixel and radius. Returns null
     * when the detection doesn't carry a known object diameter (getRadiusMM() == 0), so an
     * untyped detection never silently lands on the floor plane.
     */
    public Point calculateBallRobotFramePos(VisionRecognition recognition,
                                             Matrix camToRobot, Vector3d camPos) {
        double radiusMM = recognition.getRadiusMM();
        if (radiusMM <= 0) return null;
        return calculateBallRobotFramePos(recognition.center.getX(), recognition.center.getY(),
                radiusMM, camToRobot, camPos);
    }

    /** calculateBallRobotFramePos, then rotated/translated into field coordinates by robotPose. */
    public Point calculateBallWorldPos(VisionRecognition recognition, Matrix camToRobot,
                                        Vector3d camPos, Position robotPose) {
        return toField(calculateBallRobotFramePos(recognition, camToRobot, camPos), robotPose);
    }

    /**
     * Ranges by known real-world object size instead of floor-plane intersection.
     * Unlike calculateRobotFramePos, this works for objects not resting on the floor
     * (elevated, stacked, mid-air) since it never assumes a floor plane.
     *
     * @param objectDiameterMM real-world diameter of the (assumed circular) object,
     *                         e.g. KColorBlobProcessor.getObjectDiameterMM()
     */
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
        Vector3d dir = rayInRobotFrame(recognition.center.getX(), recognition.center.getY(), mountRotation);
        Vector3d hit = cameraOffset.add(dir.normalize().multiply(range));
        return new Point(hit.getX(), hit.getZ());
    }

    public Point calculateWorldPosFromSize(VisionRecognition recognition, double objectDiameterMM, Position robotPose) {
        return toField(calculateRobotFramePosFromSize(recognition, objectDiameterMM), robotPose);
    }

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
    public Vector3d getCameraOffset() { return cameraOffset; }

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
