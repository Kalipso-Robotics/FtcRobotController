package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Matrix;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.junit.Before;
import org.junit.Test;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Raytracer (piggyback mode) end to end: a known ball is projected through the camera model into a pixel box,
 * the box goes through the real KVisionProcessor listener hook, and the field position that
 * lands in SharedData must be the ball we started from.
 */
public class RaytracerPublishTest {

    private static final VisionConfig.Camera CAM = VisionConfig.ARDUCAM;
    private static final double POLLEN_R = VisionConfig.POLLEN_DIAMETER_MM / 2.0;

    /** Detector stand-in: feeds whatever list the test sets through processFrame. */
    private static class FakeDetector extends KVisionProcessor<List<VisionRecognition>> {
        List<VisionRecognition> next = new ArrayList<>();
        @Override protected List<VisionRecognition> detect(Mat frame) { return next; }
    }

    private FakeDetector detector;
    private Raytracer localizer;
    // SharedData's pose history is static and ignores out-of-order samples, so every test gets
    // a timestamp later than anything an earlier test recorded.
    private static long clock = System.nanoTime();
    private long nanos;

    @Before
    public void setUp() {
        nanos = clock += 10_000_000_000L;
        SharedData.resetBallInformation();
        detector = new FakeDetector();
        localizer = new Raytracer(detector, CAM);
    }

    /** Pixel box of a ball whose centre sits at robot-frame (x left, z forward), resting on the floor. */
    private static VisionRecognition project(String label, float conf, double x, double z, double r) {
        Vector3 c = cameraFrame(x, z, r);
        double psi = Math.atan2(c.x, c.z), dx = Math.asin(r / Math.hypot(c.x, c.z));
        double th = Math.atan2(c.y, c.z), dy = Math.asin(r / Math.hypot(c.y, c.z));
        double cx = CAM.cx, cy = CAM.cy, fx = CAM.fx, fy = CAM.fy;
        return new VisionRecognition(label, conf,
                (float) (cx + fx * Math.tan(psi - dx)), (float) (cy + fy * Math.tan(th - dy)),
                (float) (cx + fx * Math.tan(psi + dx)), (float) (cy + fy * Math.tan(th + dy)));
    }

    private static class Vector3 { double x, y, z; }

    /** Ball centre in the camera frame: R^T (centre - lens). */
    private static Vector3 cameraFrame(double x, double z, double r) {
        VisionConfig.Mount m = CAM.mount;
        Matrix camToRobot = Raytracer.camToRobot(m);
        double[] d = {x - m.xMM, r - m.yMM, z - m.zMM};
        Vector3 c = new Vector3();
        double[] out = new double[3];
        for (int k = 0; k < 3; k++) {
            for (int i = 0; i < 3; i++) out[k] += camToRobot.get(i, k) * d[i];
        }
        c.x = out[0]; c.y = out[1]; c.z = out[2];
        return c;
    }

    private void frame(VisionRecognition... recs) {
        detector.next = new ArrayList<>(Arrays.asList(recs));
        detector.processFrame(null, nanos);
    }

    @Test
    public void ballLandsAtItsFieldPosition_withRobotMovedAndTurned() {
        Position pose = new Position(1000, -500, Math.toRadians(30));
        SharedData.setOdometryWheelIMUPosition(pose, nanos);

        frame(project("pollen", 0.9f, 100, 800, POLLEN_R));

        List<BallInformation> balls = SharedData.getBallInformation();
        assertEquals(1, balls.size());
        Point want = Raytracer.toField(new Point(100, 800), pose);
        assertEquals(want.getX(), balls.get(0).x, 1e-3);
        assertEquals(want.getY(), balls.get(0).y, 1e-3);
        assertEquals(BallInformation.Type.YELLOW, balls.get(0).type);
        assertEquals(0.9f, balls.get(0).confidence, 1e-6);
        assertEquals(Math.hypot(100, 800), balls.get(0).distanceMM, 1e-3);
    }

    @Test
    public void atTheOriginFieldXIsForwardAndFieldYIsLeft() {
        SharedData.setOdometryWheelIMUPosition(new Position(0, 0, 0), nanos);

        frame(project("pollen", 0.9f, 120, 850, POLLEN_R)); // 120 mm left, 850 mm forward

        BallInformation b = SharedData.getBallInformation().get(0);
        assertEquals(850, b.x, 1e-3);
        assertEquals(120, b.y, 1e-3);
        assertEquals(1, localizer.getLastResult().detections.size());
    }

    @Test
    public void usesPoseAtCaptureTimeNotLatestPose() {
        Position atCapture = new Position(0, 0, 0);
        SharedData.setOdometryWheelIMUPosition(atCapture, nanos);
        SharedData.setOdometryWheelIMUPosition(new Position(500, 500, 1.0), nanos + 50_000_000L);

        frame(project("nectar_red", 0.8f, 0, 700, 35.56)); // processed "late"; captured at `nanos`

        BallInformation b = SharedData.getBallInformation().get(0);
        Point want = Raytracer.toField(new Point(0, 700), atCapture);
        assertEquals(want.getX(), b.x, 1e-3);
        assertEquals(want.getY(), b.y, 1e-3);
        assertEquals(BallInformation.Type.RED, b.type);
    }

    @Test
    public void typeMapping() {
        assertEquals(BallInformation.Type.YELLOW, Raytracer.typeFor("pollen"));
        assertEquals(BallInformation.Type.RED, Raytracer.typeFor("nectar_red"));
        assertEquals(BallInformation.Type.BLUE, Raytracer.typeFor("nectar_blue"));
        assertEquals(BallInformation.Type.YELLOW, Raytracer.typeFor("Yellow"));
        assertEquals(BallInformation.Type.UNKNOWN, Raytracer.typeFor("class32"));
    }

    @Test
    public void lowConfidenceAndClippedBoxesAreDropped_butEmptyListStillPublishes() {
        SharedData.setOdometryWheelIMUPosition(new Position(0, 0, 0), nanos);
        VisionRecognition weak = project("pollen", 0.1f, 0, 800, POLLEN_R);
        VisionRecognition clipped = new VisionRecognition("pollen", 0.9f, 0, 300, 90, 390);

        frame(weak, clipped);

        assertEquals(0, SharedData.getBallInformation().size());
        assertTrue("an empty frame must still stamp the time",
                SharedData.getBallInformationTimeMs() > 0);
        assertEquals(0, localizer.getLastResult().estimates.size());
    }

    @Test
    public void clipGateUsesTheCameraImageSize() {
        // x=1279 is inside a 1280-wide frame's margin but far outside 640x480's.
        VisionRecognition nearRightEdge = new VisionRecognition("pollen", 0.9f, 1200, 300, 1279, 390);
        assertEquals(1280, CAM.width);
        assertEquals(720, CAM.height);
        assertTrue(localizer.isClipped(nearRightEdge));
        VisionRecognition mid = new VisionRecognition("pollen", 0.9f, 700, 300, 790, 390);
        assertTrue(!localizer.isClipped(mid));
        VisionConfig.Camera small = new VisionConfig.Camera("small", 640, 480, 500, 500, 320, 240,
                0, 0, 0, 0, 0, CAM.mount);
        assertTrue(new Raytracer(small).isClipped(mid)); // 640 wide: x=790 is off-frame
    }

    @Test
    public void growParameterMatchesPixelForm_andUnknownDiameterIsNull() {
        VisionRecognition d = project("pollen", 0.9f, 50, 900, POLLEN_R);
        Raytracer.Estimate viaDet = localizer.estimate(d, POLLEN_R, 0);
        Raytracer.Estimate viaPx = localizer.estimate(d.left, d.right, d.top, d.bottom, POLLEN_R, CAM.mount);
        assertEquals(viaPx.robotPos.getY(), viaDet.robotPos.getY(), 1e-9);
        assertEquals(viaPx.robotPos.getX(), viaDet.robotPos.getX(), 1e-9);
        assertEquals(900, viaDet.robotPos.getY(), 1e-3);
        assertEquals(50, viaDet.robotPos.getX(), 1e-3);
        // the detection's own diameter is unset on a TFLite box
        assertEquals(null, localizer.estimate(d, 0));
    }

    @Test
    public void mountSupplierMovesTheBall() {
        VisionRecognition d = project("pollen", 0.9f, 0, 900, POLLEN_R);
        double before = localizer.estimate(d, POLLEN_R, 0).robotPos.getY();
        VisionConfig.Mount m = CAM.mount;
        localizer.setMountSupplier(() -> new VisionConfig.Mount(m.pitchDeg + 2, m.yawDeg, m.rollDeg, m.xMM, m.yMM, m.zMM));
        double after = localizer.estimate(d, POLLEN_R, 0).robotPos.getY();
        assertTrue("2 deg more pitch must move the ball: " + before + " -> " + after, Math.abs(after - before) > 20);
    }
}
