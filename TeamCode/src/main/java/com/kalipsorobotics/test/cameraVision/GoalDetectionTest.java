package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.cameraVision.GoalDetection;
import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Goal Detection Test")
public class GoalDetectionTest extends LinearOpMode {
    private GoalDetection goalDetection;
    @Override

    public void runOpMode() throws InterruptedException {
        goalDetection = new GoalDetection();
        while (opModeIsActive()) {
            KLog.d("Goal detection",
                    "Alliance Color: " + goalDetection.getGoalColor(goalDetection.getGoalId()) +
                    " distance: " + goalDetection.getDistanceToGoal() +
                    " heading error: " + goalDetection.getHeadingError(SharedData.getOdometryPosition().getTheta()));
        }
    }
}
