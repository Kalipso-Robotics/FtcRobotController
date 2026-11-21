package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.junit.After;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

public class ClusterDetectionActionTest {
    @After
    public void clean() {
        SharedData.resetBallInformation();
        SharedData.setTargetClusterXY(null);
    }

    private static void runToDone(ClusterDetectionAction a) {
        for (int i = 0; i < 1000 && !a.isFinished(); i++) {
            a.updateCheckDone();
        }
        assertTrue(a.isFinished());
    }

    @Test
    public void readsSharedDataOnFirstUpdateAndPublishesTarget() {
        ClusterDetectionAction a = new ClusterDetectionAction(AllianceColor.RED); // built BEFORE the balls are seen
        BallInformation.Type ours = BallCluster.allianceType(AllianceColor.RED);
        SharedData.setBallInformation(Arrays.asList(new BallInformation(300, 300, ours), new BallInformation(400, 350, ours)));
        runToDone(a);
        assertNotNull(a.getBestCluster());
        assertNotNull(SharedData.getTargetClusterXY());
    }

    @Test
    public void noBallsClearsTarget() {
        SharedData.setTargetClusterXY(new org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point(1, 1));
        SharedData.setBallInformation(Collections.emptyList());
        ClusterDetectionAction a = new ClusterDetectionAction(AllianceColor.RED);
        runToDone(a);
        assertNull(SharedData.getTargetClusterXY());
    }
}
