package com.kalipsorobotics.test.cameraVision;

import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.KMotifDetection;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MotifDetectionTest extends LinearOpMode {
    OpModeUtilities opModeUtilities;
    KMotifDetection kMotifDetection;
    ObiliskDetection obiliskDetection;
    DriveTrain driveTrain;
    DriveAction driveAction;
    @Override
    public void runOpMode() throws InterruptedException {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        obiliskDetection = new ObiliskDetection();
        kMotifDetection = new KMotifDetection(obiliskDetection);
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        driveAction = new DriveAction(driveTrain);
        waitForStart();
        while (opModeIsActive()) {
            driveAction.move(gamepad1);
            Log.d("Motif Detection", "Motif Pattern: " + kMotifDetection.getMotifPattern());
        }
    }
}
