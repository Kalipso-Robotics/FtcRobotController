package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.KMotifDetectionAction;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MotifDetectionTest extends LinearOpMode {
    OpModeUtilities opModeUtilities;
    KMotifDetectionAction kMotifDetection;
    ObiliskDetection obiliskDetection;
    DriveTrain driveTrain;
    DriveAction driveAction;
    @Override
    public void runOpMode() throws InterruptedException {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        obiliskDetection = new ObiliskDetection();
        obiliskDetection.init(hardwareMap);
        kMotifDetection = new KMotifDetectionAction(obiliskDetection);
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        driveAction = new DriveAction(driveTrain);
        waitForStart();
        while (opModeIsActive()) {
            driveAction.move(gamepad1);
            kMotifDetection.updateCheckDone();
            //Detectable from across field
            KLog.d("Motif Detection", "Motif Pattern: " + kMotifDetection.getMotifPattern());
            if (kMotifDetection.getIsDone()) {
                break;
            }
        }
        
        obiliskDetection.close();
    }
}
