package com.kalipsorobotics.test.cameraVision;

import android.util.Log;

import com.kalipsorobotics.cameraVision.GoalDetection;
import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class GoalDetectionTest extends LinearOpMode {
    private GoalDetection goalDetection;
    @Override

    public void runOpMode() throws InterruptedException {
        goalDetection = new GoalDetection();
        while (opModeIsActive()) {
            Log.d("Goal detection", "" + goalDetection.getGoalColor(goalDetection.getGoalId()) + " " + "distance: " +
                    goalDetection.getDistanceToGoal() + "heading error: " + goalDetection.getHeadingError(SharedData.getOdometryPosition().getTheta()));
        }
    }
}
