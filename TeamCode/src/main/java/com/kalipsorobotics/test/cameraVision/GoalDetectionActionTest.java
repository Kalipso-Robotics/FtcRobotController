package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class GoalDetectionActionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(this.hardwareMap, this, this.telemetry);
        Turret turret = Turret.getInstance(opModeUtilities);
        // TEST ON RED or change tagId
        AprilTagDetectionAction aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, 24);

        waitForStart();

        while (opModeIsActive()) {
            aprilTagDetectionAction.updateCheckDone();
            telemetry.addData("current limelight position ", SharedData.getLimelightPosition().toString());
        }
    }
}
