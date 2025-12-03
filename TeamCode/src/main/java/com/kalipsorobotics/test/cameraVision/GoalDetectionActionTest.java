package com.kalipsorobotics.test.cameraVision;

import android.graphics.Path;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
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
        GoalDetectionAction goalDetectionAction = new GoalDetectionAction(opModeUtilities, turret);

        waitForStart();

        while (opModeIsActive()) {
            goalDetectionAction.updateCheckDone();
            telemetry.addData("current limelight position ", SharedData.getLimelightOdometryPosition().toString());
        }
    }
}
