package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.junit.After;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ClusterDetectionActionTest {
    private static final AllianceColor RED = AllianceColor.RED;
    private static final BallInformation.Type OURS = BallInformation.Type.RED;
    private static final BallInformation.Type THEIRS = BallInformation.Type.BLUE;
    private static final BallInformation.Type POLLEN = BallInformation.Type.YELLOW;
    private static final Point ROBOT = new Point(300, 1800);

    @After
    public void clean() {
        SharedData.resetBallInformation();
        SharedData.setTargetClusterXY(null);
    }

    private static BallInformation ball(double x, double y, BallInformation.Type type) {
        return new BallInformation(x, y, type);
    }

    @Test
    public void readsSharedDataOnFirstUpdateAndPublishesTarget() {
        ClusterDetectionAction a = new ClusterDetectionAction(RED); // built BEFORE the balls are seen
        SharedData.setBallInformation(Arrays.asList(ball(300, 300, OURS), ball(400, 350, OURS)));
        a.updateCheckDone();
        assertTrue(a.isFinished());
        assertNotNull(a.getBestCluster());
        assertNotNull(SharedData.getTargetClusterXY());
    }

    @Test
    public void noBallsClearsTarget() {
        SharedData.setTargetClusterXY(new Point(1, 1));
        SharedData.setBallInformation(Collections.emptyList());
        ClusterDetectionAction a = new ClusterDetectionAction(RED);
        a.updateCheckDone();
        assertNull(SharedData.getTargetClusterXY());
    }

    @Test
    public void ballsAcrossATileLineAreOneGroup() {
        // 609.6 is a tile line: the old tile bucketing split these two touching balls.
        List<BallCluster> groups = ClusterDetectionAction.group(Arrays.asList(ball(600, 1000, OURS), ball(620, 1000, OURS)), RED);
        assertEquals(1, groups.size());
        assertEquals(2, groups.get(0).size());
    }

    @Test
    public void aLineOfBallsChainsIntoOneGroup() {
        List<BallCluster> groups = ClusterDetectionAction.group(Arrays.asList(
                ball(500, 1000, POLLEN), ball(700, 1000, POLLEN), ball(900, 1000, POLLEN), ball(1100, 1000, POLLEN)), RED);
        assertEquals(1, groups.size());
        assertEquals(4, groups.get(0).size());
    }

    @Test
    public void groupWithAnOpponentBallIsDropped() {
        List<BallCluster> groups = ClusterDetectionAction.group(Arrays.asList(
                ball(500, 1000, OURS), ball(600, 1000, THEIRS), ball(500, 2500, POLLEN)), RED);
        assertEquals(1, groups.size());
        assertEquals(POLLEN, groups.get(0).getBalls().get(0).type);
    }

    @Test
    public void ballsOnTheOtherHalfAreIgnored() {
        assertNull(ClusterDetectionAction.chooseBest(Collections.singletonList(ball(FieldConfig.MIDLINE_X + 100, 1800, OURS)), ROBOT, RED, 4));
    }

    @Test
    public void noRoomPicksNothing() {
        assertNull(ClusterDetectionAction.chooseBest(Collections.singletonList(ball(600, 1800, OURS)), ROBOT, RED, 0));
    }

    @Test
    public void nectarBeatsTheSameNumberOfPollenAtTheSameDistance() {
        BallCluster best = ClusterDetectionAction.chooseBest(Arrays.asList(
                ball(800, 1300, POLLEN), ball(800, 2300, OURS)), ROBOT, RED, 4);
        assertEquals(OURS, best.getBalls().get(0).type);
    }

    @Test
    public void biggerGroupBeatsACloserSingleBall() {
        BallCluster best = ClusterDetectionAction.chooseBest(Arrays.asList(
                ball(600, 1800, POLLEN),
                ball(1200, 600, OURS), ball(1300, 600, OURS), ball(1250, 680, OURS)), ROBOT, RED, 4);
        assertEquals(3, best.size());
    }

    @Test
    public void roomCapsTheValueOfABigGroup() {
        // With room for 1, a far group of 4 is worth no more than a near single ball: take the near one.
        BallCluster best = ClusterDetectionAction.chooseBest(Arrays.asList(
                ball(600, 1800, OURS),
                ball(1500, 300, OURS), ball(1600, 300, OURS), ball(1500, 400, OURS), ball(1600, 400, OURS)), ROBOT, RED, 1);
        assertEquals(1, best.size());
    }

    @Test
    public void aimsAtTheNearestBallWhenTheMiddleIsEmpty() {
        // Two balls 240 mm apart chain into one group; the middle is 120 mm from each, so aim at a ball.
        BallCluster group = ClusterDetectionAction.group(Arrays.asList(ball(500, 1000, OURS), ball(740, 1000, OURS)), RED).get(0);
        Point aim = ClusterDetectionAction.aimPointOf(group);
        assertTrue(aim.getX() == 500 || aim.getX() == 740);
    }

    @Test
    public void pointsTakeNectarFirst() {
        assertEquals(2 * BallCluster.NECTAR_POINTS + BallCluster.POLLEN_POINTS, BallCluster.points(2, 5, 3));
        assertEquals(0, BallCluster.points(3, 3, 0));
    }
}
