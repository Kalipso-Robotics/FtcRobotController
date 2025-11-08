package com.kalipsorobotics.test.cameraVision;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.actions.cameraVision.MotifDetectionAction;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class MotifDetectionTest extends LinearOpMode {
    OpModeUtilities opModeUtilities;
    MotifDetectionAction kMotifDetection;
    MotifCamera motifCamera;
    DriveTrain driveTrain;
    DriveAction driveAction;
    @Override
    public void runOpMode() throws InterruptedException {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        motifCamera = new MotifCamera(opModeUtilities);
        kMotifDetection = new MotifDetectionAction(motifCamera);
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        waitForStart();
        while (opModeIsActive()) {
            driveTrain.setPower(0.4);
            kMotifDetection.updateCheckDone();
            //Detectable from across field
            KLog.d("Motif Detection", "Motif Pattern: " + kMotifDetection.getMotifPattern());
            if (kMotifDetection.getIsDone()) {
                break;
            }
        }
        kMotifDetection.close();
    }
}
