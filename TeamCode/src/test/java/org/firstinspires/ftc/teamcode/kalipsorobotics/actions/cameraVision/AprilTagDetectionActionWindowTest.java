package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

/**
 * Checks the sliding window in AprilTagDetectionAction without hardware: frames are fed in by hand with a made-up clock.
 */
public class AprilTagDetectionActionWindowTest {

    private static final long MS = 1_000_000L; // nanoseconds in one millisecond

    /** A camera that does nothing (the window tests never ask it for frames). */
    private static final AprilTagCamera NO_CAMERA = new AprilTagCamera() {
        public void start() { }
        public void stop() { }
        public List<TagObservation> getLatestObservations() { return Collections.emptyList(); }
    };

    private static AprilTagDetectionAction newAction() {
        return new AprilTagDetectionAction(null, null, AllianceColor.RED, NO_CAMERA);
    }

    private static HiveTagLocalizer.Fix fix(double x, double y, HiveTagLocalizer.Cell upCell, boolean tipping) {
        return new HiveTagLocalizer.Fix(new Point(x, y), upCell, tipping, 4);
    }

    @Test
    public void threeAgreeingFrames_giveTheUpCell() {
        AprilTagDetectionAction action = newAction();
        Position odometry = new Position(0, 0, 0);
        for (int i = 0; i < 3; i++) action.addToWindow(fix(0, 0, HiveTagLocalizer.Cell.A, false), true, i * 30 * MS, odometry);

        assertEquals(HiveTagLocalizer.Cell.A, action.windowUpCell(100 * MS));
    }

    @Test
    public void oldFrames_dropOutOfTheWindow() {
        AprilTagDetectionAction action = newAction();
        Position odometry = new Position(0, 0, 0);
        for (int i = 0; i < 3; i++) action.addToWindow(fix(500, 500, HiveTagLocalizer.Cell.A, true), true, i * 30 * MS, odometry);
        assertTrue(action.windowTipping(100 * MS));
        assertNotNull(action.windowPosition(100 * MS, odometry));

        long muchLater = 2000 * MS; // the robot looked away for two seconds
        assertFalse(action.windowTipping(muchLater));
        assertNull(action.windowPosition(muchLater, odometry));
        assertNull(action.windowUpCell(muchLater));
    }

    @Test
    public void positionFollowsOdometry_whileDriving() {
        AprilTagDetectionAction action = newAction();
        // Both frames say odometry is 100 mm short in x (camera 1000, odometry 900, then camera 1100, odometry 1000).
        action.addToWindow(fix(1000, 2000, null, false), true, 0, new Position(900, 2000, 0));
        action.addToWindow(fix(1100, 2000, null, false), true, 20 * MS, new Position(1000, 2000, 0));

        // Robot has driven on: odometry now says 1200. The fix should be 1300, not the old average 1050.
        Position published = action.windowPosition(40 * MS, new Position(1200, 2000, 0.5));
        assertNotNull(published);
        assertEquals(1300, published.getX(), 0.001);
        assertEquals(2000, published.getY(), 0.001);
        assertEquals(0.5, published.getTheta(), 0.001); // heading stays from odometry
    }
}
