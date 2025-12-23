package com.kalipsorobotics.test.localization;

import com.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.turret.TurretAutoAlignLimelight;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryFileWriter;
import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class OdometryEncoderCalcTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        DriveAction driveAction = new DriveAction(driveTrain);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        OdometryFileWriter odometryFileWriter = new OdometryFileWriter("OdometryTest", opModeUtilities);


        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        Position pos1 = SharedData.getOdometryWheelIMUPosition();
        Position pos2 = SharedData.getOdometryWheelIMUPosition();
        double power = 0;
        boolean calibrationComplete = false;
        String action;

        Turret.setInstanceNull();
        Turret turret = Turret.getInstance(opModeUtilities);

        AprilTagDetectionAction aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, 24);

        waitForStart();
        try {
            while (opModeIsActive()) {

//            while (SharedData.getOdometryPositionMap().get(OdometrySensorCombinations.WHEEL_IMU).getCurrentVelocity().getX() <  (50.0/1000.0) && !calibrationComplete) {
//                driveTrain.setPower(power);
//                power += 0.001;
//            }
//            calibrationComplete = true;
//            KLog.d("minimumPower", "minimum power " + power);
//            driveTrain.setPower(power);
                aprilTagDetectionAction.updateCheckDone();

                if (gamepad1.a) {
                    action = "Slamming";
                } else {
                    action = "normal";
                }

                odometryFileWriter.writeOdometryPositionHistory(SharedData.getOdometryPositionMap(), action);
                driveAction.move(gamepad1);


                KLog.d("Left_Encoder", "left_encoder " + driveTrain.getLeftEncoder().getCurrentPosition());
                if (driveTrain.getfLeftPower() == 0) {
                    KLog.d("Odometry_Position_While_Stopped", SharedData.getOdometryWheelIMUPosition() + ", power: fleft " + driveTrain.getfLeftPower() + ", fright " + driveTrain.getfRightPower() + ", bleft " + driveTrain.getbLeftPower() + ", bright " + driveTrain.getbRightPower());
                    pos1 = SharedData.getOdometryWheelIMUPosition();
                } else {
                    KLog.d("Odometry_Position_While_Moving", SharedData.getOdometryWheelIMUPosition() + ", power: fleft " + driveTrain.getfLeftPower() + ", fright " + driveTrain.getfRightPower() + ", bleft " + driveTrain.getbLeftPower() + ", bright " + driveTrain.getbRightPower());
                    pos2 = SharedData.getOdometryWheelIMUPosition();
                }
                KLog.d("Calibration_Data", "nurture distance " + pos1.distanceTo(pos2) + " nurture angle " + Math.abs(pos1.getTheta() - pos2.getTheta()));
                KLog.d("encoders", "count back: " + odometry.getBackEncoderMM() +
                        "  count right: " + odometry.getRightEncoderMM() +
                        "  count left: " + odometry.getLeftEncoderMM());
                //KLog.d("Velocity", Objects.requireNonNull(SharedData.getOdometryPositionMap().get(OdometrySensorCombinations.WHEEL_IMU)).getCurrentVelocity().toString());
            }
        } finally {
            odometryFileWriter.close();
            OpModeUtilities.shutdownExecutorService(executorService);
        }

    }
}