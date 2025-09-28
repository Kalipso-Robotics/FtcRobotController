package com.kalipsorobotics.test.cameraVision;

import android.util.Log;

import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.kalipsorobotics.utilities.KMotifDetectionAction;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoDriveWithMotifDetection extends LinearOpMode {
    KMotifDetectionAction motifDetection;
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        PurePursuitAction moveForward = new PurePursuitAction(driveTrain);
        moveForward.addPoint(500, 0, 0);
        moveForward.setMaxTimeOutMS(8000);

        waitForStart();
        while(opModeIsActive()) {
            moveForward.updateCheckDone();
            motifDetection.updateCheckDone();
            Log.d("Motif Detection Auto", "Motif Pattern: " + motifDetection.getMotifPattern());
            telemetry.addLine("" + motifDetection.getMotifPattern());
        }
    }
}
