package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import java.util.Locale;

/**
 * Drives both odometry algorithms side by side and scores them the same way: bring the robot
 * back to where it started and see how far off each one thinks you are.
 *
 * Protocol: tape a mark on the floor, press START, drive a lap with gamepad1 (field-oriented),
 * put the robot back on the mark, press A. Repeat ~10 times with different laps (straight,
 * strafe, spin) -- each A press writes one TRIAL row and re-zeros both algorithms for the next
 * lap, all in one CSV. B re-zeros without recording (use it if you messed up a lap).
 *
 * Filter the CSV for lines starting with "TRIAL" and compare octo_closure_mm vs
 * old_closure_mm across trials (mean, and mm of error per metre driven).
 *
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp
public class CompareOdometryTest extends LinearOpMode {

    private final OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock enc = new OctoQuad.EncoderDataBlock();
    private final OctoConfig.Pose octoPose = new OctoConfig.Pose();
    private final ElapsedTime runtime = new ElapsedTime();

    private OctoQuad q;
    private KFileWriter csv;
    private OpModeUtilities opModeUtilities;
    private DriveTrain driveTrain;
    private IMUModule imuModule;
    private Odometry odometry;
    private DcMotor intake;

    private double pathMm, turnedDeg;
    private double prevX, prevY;
    private boolean prevSeeded;
    private double lastRawHeadingRad;
    private boolean headingSeeded;
    private int trialNum;
    private double speed = 1;

    @Override
    public void runOpMode() {
        q = hardwareMap.get(OctoQuad.class, OctoConfig.HARDWARE_NAME);
        intake = hardwareMap.get(DcMotor.class, "intake");

        OctoConfig.apply(q);

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule.setInstanceNull();
        imuModule = IMUModule.getInstance(opModeUtilities);
        Odometry.setInstanceNull();
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        csv = new KFileWriter("CompareOdometry", opModeUtilities);
        csv.writeLine("type,t_s,trial,octo_x,octo_y,octo_hdg_deg,old_x,old_y,old_hdg_deg,"
                + "path_mm,turned_deg,octo_closure_mm,octo_hdg_err_deg,old_closure_mm,old_hdg_err_deg,crcOk,loop_ms");

        telemetry.addLine("COMPARE ODOMETRY -- old vs OctoQuad");
        telemetry.addLine("Mark the robot's spot. Drive a lap, come back, press A to log it.");
        telemetry.addLine("B re-zeros without recording. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) { csv.close(); return; }

        if (!OctoConfig.calibrateImu(q, this)) {
            telemetry.addLine("IMU calibration failed. Not a usable test.");
            telemetry.update();
            csv.writeLine("# ABORT imu calibration failed");
            csv.close();
            sleep(4000);
            return;
        }

        FieldOrientedDriveAction drive = new FieldOrientedDriveAction(driveTrain, imuModule);

        q.readLocalizerDataAndAllEncoderData(loc, enc);
        zero();

        ElapsedTime loopTimer = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                double loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                q.readLocalizerDataAndAllEncoderData(loc, enc);
                if (loc.isDataValid() && enc.isDataValid()) {
                    OctoConfig.toRobotFrame(loc, octoPose);
                    updateOdometer(loc.heading_rad);
                }
                Position oldPos = odometry.update();

                drive.move(gamepad1);

                if (gamepad1.bWasPressed()) {
                    zero();
                } else if (gamepad1.aWasPressed()) {
                    logTrial(oldPos, loopMs);
                    zero();
                }

                if (gamepad1.right_trigger > 0.1) {
                    intake.setPower(-speed);
                } else if (gamepad1.right_bumper) {
                    intake.setPower(speed);
                } else {
                    intake.setPower(0);
                }

                if (gamepad1.dpad_down) {
                    speed-=0.1;
                } else if (gamepad1.dpad_up) {
                    speed+=0.1;
                }

                render(oldPos);
                telemetry.addLine("Speed: " + speed);
                telemetry.update();

                KLog.d("odo_compare", () -> String.format(Locale.US,
                        "octo x %.1f y %.1f h %.2f | old x %.1f y %.1f h %.2f",
                        octoPose.x, octoPose.y, octoPose.headingDeg,
                        oldPos.getX(), oldPos.getY(), Math.toDegrees(oldPos.getTheta())));
            }
        } finally {
            csv.close();
        }
    }

    private void render(Position oldPos) {
        telemetry.addLine("--- octo ---");
        telemetry.addData("x / y / h", "%8.1f / %8.1f mm / %7.2f deg",
                octoPose.x, octoPose.y, octoPose.headingDeg);
        telemetry.addLine("--- old (wheel+imu) ---");
        telemetry.addData("x / y / h", "%8.1f / %8.1f mm / %7.2f deg",
                oldPos.getX(), oldPos.getY(), Math.toDegrees(oldPos.getTheta()));
        telemetry.addLine();
        telemetry.addData("trials logged", trialNum);
        telemetry.addData("this lap", "%.1f m driven, %.0f deg turned", pathMm / 1000.0, turnedDeg);
        telemetry.addLine();
        telemetry.addLine("A = log trial + re-zero.  B = re-zero only.");
    }

    /** One TRIAL summary row: each algorithm's return-to-start error for this lap. */
    private void logTrial(Position oldPos, double loopMs) {
        trialNum++;
        double octoClosure = Math.hypot(octoPose.x, octoPose.y);
        double octoHdgErr = Math.abs(wrapDeg(octoPose.headingDeg));
        double oldClosure = Math.hypot(oldPos.getX(), oldPos.getY());
        double oldHdgErr = Math.abs(wrapDeg(Math.toDegrees(oldPos.getTheta())));

        csv.writeLine(String.format(Locale.US,
                "TRIAL,%.3f,%d,%.2f,%.2f,%.4f,%.2f,%.2f,%.4f,%.1f,%.2f,%.2f,%.4f,%.2f,%.4f,%b,%.2f",
                runtime.seconds(), trialNum,
                octoPose.x, octoPose.y, octoPose.headingDeg,
                oldPos.getX(), oldPos.getY(), Math.toDegrees(oldPos.getTheta()),
                pathMm, turnedDeg,
                octoClosure, octoHdgErr, oldClosure, oldHdgErr,
                loc.crcOk, loopMs));
        try {
            csv.flush();
        } catch (java.io.IOException e) {
            telemetry.addLine("CSV flush failed: " + e.getMessage());
        }
    }

    /** Path length and total angle turned this lap, from the octo pose (same as OctoTest). */
    private void updateOdometer(double rawHeadingRad) {
        if (prevSeeded) {
            pathMm += Math.hypot(octoPose.x - prevX, octoPose.y - prevY);
        }
        prevX = octoPose.x;
        prevY = octoPose.y;
        prevSeeded = true;

        if (!headingSeeded) {
            lastRawHeadingRad = rawHeadingRad;
            headingSeeded = true;
            return;
        }
        double d = OctoConfig.wrapDeltaRad(rawHeadingRad, lastRawHeadingRad);
        lastRawHeadingRad = rawHeadingRad;
        turnedDeg += Math.abs(Math.toDegrees(d));
    }

    private static double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }

    /** Re-zero both algorithms and the IMU yaw they both key off of, plus this lap's odometer. */
    private void zero() {
        q.setLocalizerPose(0, 0, 0f);
        octoPose.x = octoPose.y = octoPose.headingDeg = 0;
        prevSeeded = false;
        headingSeeded = false;
        pathMm = turnedDeg = 0;

        imuModule.getIMU().resetYaw();
        Odometry.setInstanceNull();
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        csv.writeLine("# --- zeroed at " + String.format(Locale.US, "%.1f s", runtime.seconds()));
    }
}
