package org.firstinspires.ftc.teamcode.kalipsorobotics.test.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometryFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Disabled
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

        AprilTagDetectionAction aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, 24, AllianceColor.RED);

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
                KLog.d("OdoEncoderCalcTest", "Successfully switched color to blue " + aprilTagDetectionAction);
                if (gamepad1.a) {
                    action = "Slamming";
                } else {
                    action = "normal";
                }

                if (gamepad1.x) {
                    aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, 20, AllianceColor.BLUE);
                    KLog.d("OdoEncoderCalcTest", "Successfully switched color to blue " + aprilTagDetectionAction);
                }
                if (gamepad1.b) {
                    aprilTagDetectionAction = new AprilTagDetectionAction(opModeUtilities, turret, 24, AllianceColor.RED);
                    KLog.d("OdoEncoderCalcTest", "Successfully switched color to red " + aprilTagDetectionAction);
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
                KLog.d("encodersMM", "count back: " + odometry.getBackEncoderMM() +
                        "  count right: " + odometry.getRightEncoderMM() +
                        "  count left: " + odometry.getLeftEncoderMM());
                KLog.d("encoderTicks", "count back ticks: " + odometry.getCurrentBackTicks() + " count right ticks: " + odometry.getCurrentRightTicks() + " count left ticks: " + odometry.getCurrentLeftTicks());
                //KLog.d("Velocity", Objects.requireNonNull(SharedData.getOdometryPositionMap().get(OdometrySensorCombinations.WHEEL_IMU)).getCurrentVelocity().toString());
            }
        } finally {
            odometryFileWriter.close();
            OpModeUtilities.shutdownExecutorService(executorService);
        }

    }
}