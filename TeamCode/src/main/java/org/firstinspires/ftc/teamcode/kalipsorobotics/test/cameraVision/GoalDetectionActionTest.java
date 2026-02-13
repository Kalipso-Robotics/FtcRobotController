package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
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
        AprilTagDetectionAction aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, 24, AllianceColor.RED);

        waitForStart();

        while (opModeIsActive()) {
            aprilTagDetectionAction.updateCheckDone();
            telemetry.addData("current limelight position ", SharedData.getLimelightRawPosition().toString());
        }
    }
}
