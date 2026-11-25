package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallCluster;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;

/** The look/choose loop without hardware (it never drives here: the tests stop before the pickup runs). */
public class VisionPickupActionTest {
    private double savedTimeout;

    @Before
    public void setUp() {
        savedTimeout = VisionPickupAction.LOOK_TIMEOUT_MS;
        SharedData.resetBallInformation();
        SharedData.resetBeamBreakData();
        SharedData.setTargetClusterXY(null);
    }

    @After
    public void tearDown() {
        VisionPickupAction.LOOK_TIMEOUT_MS = savedTimeout;
        SharedData.resetBallInformation();
        SharedData.setTargetClusterXY(null);
    }

    @Test
    public void nothingSeen_endsWithoutDriving() {
        VisionPickupAction.LOOK_TIMEOUT_MS = 0; // the look ends at once
        VisionPickupAction action = new VisionPickupAction(null, null, null, AllianceColor.RED);
        action.updateCheckDone();
        assertTrue(action.getIsDone());
        assertEquals(VisionPickupAction.EndReason.NOTHING_TO_PICK, action.getEndReason());
        assertEquals(0, action.getStops());
    }

    @Test
    public void lookWaitsForFramesThenChoosesAStop() {
        VisionPickupAction.LOOK_TIMEOUT_MS = 60_000;
        VisionPickupAction action = new VisionPickupAction(null, null, null, AllianceColor.RED);
        action.updateCheckDone(); // starts the look; no frame yet
        assertEquals(0, action.getStops());

        for (int frame = 0; frame < VisionPickupAction.LOOK_FRAMES && action.getStops() == 0; frame++) {
            sleepMs(2); // each frame gets a new timestamp
            SharedData.setBallInformation(Collections.singletonList(new BallInformation(700, 1800, BallInformation.Type.RED)));
            action.updateCheckDone();
        }
        assertEquals("chose the ball and started driving", 1, action.getStops());
        assertFalse(action.getIsDone());
        assertEquals(700, SharedData.getTargetClusterXY().getX(), 1e-9);
    }

    @Test
    public void droppingTheActionMarksItDropped() {
        VisionPickupAction action = new VisionPickupAction(null, null, null, AllianceColor.RED);
        action.setIsDone(true);
        assertEquals(VisionPickupAction.EndReason.DROPPED, action.getEndReason());
    }

    @Test
    public void throughPile_reachesTheFarthestBall() {
        BallCluster pile = new BallCluster(0, 0);
        pile.addBall(new BallInformation(700, 1800, BallInformation.Type.RED));
        pile.addBall(new BallInformation(900, 1800, BallInformation.Type.RED));
        pile.addBall(new BallInformation(800, 1950, BallInformation.Type.YELLOW));
        assertEquals(200, VisionPickupAction.throughPileMm(pile, new Point(700, 1800)), 1e-9);
    }

    @Test
    public void midline_marginCountsTheBodyCornersNotTheCenter() {
        double limit = FieldConfig.MIDLINE_X - AutoToBallsPathingAction.AUTO_MIDLINE_MARGIN_MM;
        // facing +x: the front corners lead by FRONT_MM
        assertFalse(AutoToBallsPathingAction.nearMidline(new Point(limit - RobotFootprint.FRONT_MM - 1, 1800), 0));
        assertTrue(AutoToBallsPathingAction.nearMidline(new Point(limit - RobotFootprint.FRONT_MM + 1, 1800), 0));
        // facing +y the same center is fine: only half the width sticks out toward the line
        assertFalse(AutoToBallsPathingAction.nearMidline(new Point(limit - RobotFootprint.FRONT_MM + 1, 1800), Math.PI / 2));
        assertTrue(AutoToBallsPathingAction.nearMidline(new Point(FieldConfig.MIDLINE_X + 10, 1800), Math.PI / 2));
    }

    private static void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
