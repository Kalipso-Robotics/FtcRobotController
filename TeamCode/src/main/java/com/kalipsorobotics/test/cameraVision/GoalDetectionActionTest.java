package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GoalDetectionActionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(this.hardwareMap, this, this.telemetry);
        Turret turret = Turret.getInstance(opModeUtilities);
        // TEST ON RED or change tagId
        GoalDetectionAction goalDetectionAction = new GoalDetectionAction(opModeUtilities, turret, 24);

        waitForStart();

        while (opModeIsActive()) {
            goalDetectionAction.updateCheckDone();
            telemetry.addData("current limelight position ", SharedData.getLimelightPosition().toString());
        }
    }
}
