package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
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
 * "Old" here is NOT localization.Odometry: on this test robot all three pods are wired only to
 * the OctoQuad, so Odometry's own hub-motor-port encoder reads are stuck at zero and it never
 * leaves (0,0,0). Instead, LegacyOdometry below re-runs Odometry's WHEEL_IMU math (see its
 * javadoc) against the OctoQuad's raw pod counts and the hub IMU -- same algorithm, same
 * sensors as the OctoQuad localizer, different (non-board) arithmetic. That is the actual
 * comparison this OpMode is for: two ways of turning the same three encoders + IMU into a pose.
 *
 * Protocol: tape a mark on the floor, press START and leave the robot still for IMU calibration.
 * Put the robot on the mark, press B to zero both algorithms to (0,0,0) and start a lap -- B
 * works at any time and always re-zeros, so press it again if a lap goes wrong. Drive a lap with
 * gamepad1, put the robot back on the mark, press A to log one TRIAL row and stop. Repeat ~10
 * times with different laps (straight, strafe, spin), all in one CSV.
 *
 * Filter the CSV for lines starting with "TRIAL" and compare octo_closure_mm vs
 * old_closure_mm across trials (mean, and mm of error per metre driven).
 *
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp
public class CompareOdometryTest extends LinearOpMode {

    /** Perpendicular pod (port 1) forward of robot centre; negative = behind. Solved from the
     *  2026-09-26 OctoTune spins (backMM / theta over 10 turns: -95.57 CW, -95.18 CCW), same
     *  measurement as OctoConfig.TCP_OFFSET_MM_X with the board's sign convention removed. */
    private static final double PERP_POD_FWD_MM = -95.4;

    private final OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock enc = new OctoQuad.EncoderDataBlock();
    private final OctoConfig.Pose octoPose = new OctoConfig.Pose();
    private final OctoConfig.Pose legacyPose = new OctoConfig.Pose();
    private final LegacyOdometry legacy = new LegacyOdometry();
    private final ElapsedTime runtime = new ElapsedTime();

    private OctoQuad q;
    private KFileWriter csv;
    private OpModeUtilities opModeUtilities;
    private DriveTrain driveTrain;
    private IMUModule imuModule;
//    private DcMotor intake;

    private double pathMm, turnedDeg;
    private double prevX, prevY;
    private boolean prevSeeded;
    private double lastRawHeadingRad;
    private boolean headingSeeded;
    private int trialNum;
    private boolean running;
    private double speed = 1;
    /** Hub IMU, read once per loop (and in zero()) so every consumer sees the same sample. */
    private YawPitchRollAngles ypr;
    private double hubYawZeroDeg;

    // Raw pod counts are zeroed in SOFTWARE, same trick as OctoTune: setLocalizerPose() resets
    // the board's pose accumulator, not the raw encoder count registers.
    private int rawXOffset, rawX2Offset, rawYOffset;

    @Override
    public void runOpMode() {
        q = hardwareMap.get(OctoQuad.class, OctoConfig.HARDWARE_NAME);
//        intake = hardwareMap.get(DcMotor.class, "intake");

        OctoConfig.apply(q);

        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule.setInstanceNull();
        imuModule = IMUModule.getInstance(opModeUtilities);
        // This robot's Control Hub is logo-UP, not the comp robot's DrivetrainConfig mount (logo
        // BACKWARD): the 2026-10-01 logs read pitch -91.5 deg while flat, which gimbal-locks yaw.
        // USB direction only shifts the yaw zero, which zero() re-seeds, so any horizontal one works.
        boolean hubImuOk = imuModule.getIMU().initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        csv = new KFileWriter("CompareOdometry", opModeUtilities);
        csv.writeLine("# old = LegacyOdometry (Odometry.java WHEEL_IMU math) on OctoQuad pod counts + hub IMU");
        csv.writeLine("# hub imu mount: logo UP, usb RIGHT" + (hubImuOk ? "" : " -- hub imu reinit failed"));
        csv.writeLine("type,t_s,trial,octo_x,octo_y,octo_hdg_deg,old_x,old_y,old_hdg_deg,"
                + "path_mm,turned_deg,octo_closure_mm,octo_hdg_err_deg,old_closure_mm,old_hdg_err_deg,crcOk,loop_ms");
        csv.writeLine("# LOOP rows (every loop): LOOP,t_s,running,octo_x,octo_y,octo_hdg_deg,old_x,old_y,old_hdg_deg,"
                + "pod_hdg_deg,hub_yaw_deg,hub_pitch_deg,hub_roll_deg,old_src,intake_pwr,loop_ms");

        telemetry.addLine("COMPARE ODOMETRY");
        telemetry.addLine("After START: B = zero + start lap (any time). A on the mark = log + stop.");
        telemetry.addLine("Keep the robot still after START (IMU cal).");
        while (opModeInInit()) {
            YawPitchRollAngles a = imuModule.getIMU().getRobotYawPitchRollAngles();
            telemetry.addLine("COMPARE ODOMETRY. B = zero + start lap, A on the mark = log + stop.");
            if (!hubImuOk) telemetry.addLine("!!! hub IMU reinit FAILED -- legacy heading invalid");
            telemetry.addData("hub pitch / roll (flat robot: both ~0)", "%.1f / %.1f deg",
                    a.getPitch(), a.getRoll());
            telemetry.update();
        }

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

        DriveAction drive = new DriveAction(driveTrain);

        ElapsedTime loopTimer = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                double loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                ypr = imuModule.getIMU().getRobotYawPitchRollAngles();
                q.readLocalizerDataAndAllEncoderData(loc, enc);
                if (loc.isDataValid() && enc.isDataValid()) {
                    OctoConfig.toRobotFrame(loc, octoPose);
                    if (running) updateOdometer(loc.heading_rad);
                    legacy.update(rightMM(), leftMM(), backMM(), getImuHeadingRad());
                    legacy.toPose(legacyPose);
                }

                drive.move(gamepad1);

                if (gamepad1.bWasPressed()) {
                    if (running) {
                        csv.writeLine(String.format(Locale.US,
                                "# lap restarted at %.1f s", runtime.seconds()));
                    }
                    zero();
                    running = true;
                } else if (gamepad1.aWasPressed()) {
                    if (running) {
                        logTrial(loopMs);
                        running = false;
                    }
                }

//                if (gamepad1.right_trigger > 0.1) {
//                    intake.setPower(-speed);
//                } else if (gamepad1.right_bumper) {
//                    intake.setPower(speed);
//                } else {
//                    intake.setPower(0);
//                }

                if (gamepad1.dpadDownWasPressed()) {
                    speed = MathFunctions.clamp(speed - 0.1, 0, 1);
                } else if (gamepad1.dpadUpWasPressed()) {
                    speed = MathFunctions.clamp(speed + 0.1, 0, 1);
                }

                logLoop(loopMs);
                render();
                telemetry.addLine("Speed: " + speed);
                telemetry.update();

                KLog.d("odo_compare", () -> String.format(Locale.US,
                        "octo x %.1f y %.1f h %.2f | old x %.1f y %.1f h %.2f",
                        octoPose.x, octoPose.y, octoPose.headingDeg,
                        legacyPose.x, legacyPose.y, legacyPose.headingDeg));
            }
        } finally {
            csv.close();
        }
    }

    private void render() {
        telemetry.addLine("--- octo ---");
        telemetry.addData("x / y / h", "%8.1f / %8.1f mm / %7.2f deg",
                octoPose.x, octoPose.y, octoPose.headingDeg);
        telemetry.addLine("--- old (LegacyOdometry on octo counts) ---");
        telemetry.addData("x / y / h", "%8.1f / %8.1f mm / %7.2f deg",
                legacyPose.x, legacyPose.y, legacyPose.headingDeg);
        telemetry.addLine("--- heading sources ---");
        telemetry.addData("hub pitch / roll (want ~0 flat)", "%.1f / %.1f deg",
                ypr.getPitch(), ypr.getRoll());
        telemetry.addData("octo - pod hdg", "%.2f deg", MathFunctions.angleWrapDeg(
                octoPose.headingDeg - podHeadingDeg()));
        telemetry.addLine();
        telemetry.addData("trials logged", trialNum);
        telemetry.addData("this lap", "%.1f m driven, %.0f deg turned", pathMm / 1000.0, turnedDeg);
        telemetry.addLine();
        telemetry.addLine(running
                ? ">>> LAP RUNNING <<<  back on the mark -> A = log + stop.  B = restart (re-zero)."
                : "IDLE. Robot on the mark -> B = zero + start.");
    }

    private double podHeadingDeg() {
        return OctoConfig.monitorHeadingDeg(enc.positions[OctoConfig.CH_X] - rawXOffset,
                enc.positions[OctoConfig.CH_X2] - rawX2Offset);
    }

    /** One LOOP row per loop, idle or not, so stationary drift is captured too. */
    private void logLoop(double loopMs) {
        csv.writeLine(String.format(Locale.US,
                "LOOP,%.3f,%b,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%s,%.2f,%.2f",
                runtime.seconds(), running,
                octoPose.x, octoPose.y, octoPose.headingDeg,
                legacyPose.x, legacyPose.y, legacyPose.headingDeg,
                podHeadingDeg(), ypr.getYaw() - hubYawZeroDeg, ypr.getPitch(), ypr.getRoll(),
                legacy.usedWheel ? "WHEEL" : "IMU", -1.0, loopMs));
    }

    /** One TRIAL summary row: each algorithm's return-to-start error for this lap. */
    private void logTrial(double loopMs) {
        trialNum++;
        double octoClosure = Math.hypot(octoPose.x, octoPose.y);
        double octoHdgErr = Math.abs(MathFunctions.angleWrapDeg(octoPose.headingDeg));
        double oldClosure = Math.hypot(legacyPose.x, legacyPose.y);
        double oldHdgErr = Math.abs(MathFunctions.angleWrapDeg(legacyPose.headingDeg));

        csv.writeLine(String.format(Locale.US,
                "TRIAL,%.3f,%d,%.2f,%.2f,%.4f,%.2f,%.2f,%.4f,%.1f,%.2f,%.2f,%.4f,%.2f,%.4f,%b,%.2f",
                runtime.seconds(), trialNum,
                octoPose.x, octoPose.y, octoPose.headingDeg,
                legacyPose.x, legacyPose.y, legacyPose.headingDeg,
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

    private double getImuHeadingRad() {
        // Same convention as Odometry.getIMUHeading(): negated so CW is positive.
        return -Math.toRadians(ypr.getYaw());
    }

    /** Right parallel pod (port 0), forward-positive, same frame as Odometry's right encoder. */
    private double rightMM() {
        return (enc.positions[OctoConfig.CH_X] - rawXOffset) / OctoConfig.COUNTS_PER_MM_X;
    }

    /** Left parallel pod (port 2, monitor channel), forward-positive. */
    private double leftMM() {
        return (enc.positions[OctoConfig.CH_X2] - rawX2Offset) / OctoConfig.COUNTS_PER_MM_X2;
    }

    /** Perpendicular pod (port 1), converted to +Y-right to match the project frame -- same
     *  sign flip as OctoConfig.toRobotFrame applies to the board's posY_mm. */
    private double backMM() {
        double sign = OctoConfig.MIRROR_BOARD_FRAME ? -1.0 : 1.0;
        return sign * (enc.positions[OctoConfig.CH_Y] - rawYOffset) / OctoConfig.COUNTS_PER_MM_Y;
    }

    /**
     * Put both algorithms at (0,0,0) and clear this lap's odometer.
     *
     * No IMU resetYaw: LegacyOdometry seeds prevImuHeadingRad from the current read below, so
     * there is no separate yaw-zero step to inject latency into.
     */
    private void zero() {
        q.setLocalizerPose(0, 0, 0f);
        // The board can return the old pose for a read or two after the write; wait for (0,0)
        // so the odometer's first sample isn't the whole previous lap.
        ElapsedTime t = new ElapsedTime();
        do {
            q.readLocalizerDataAndAllEncoderData(loc, enc);
        } while (opModeIsActive() && t.milliseconds() < 200
                && !(loc.isDataValid() && Math.abs(loc.posX_mm) < 2 && Math.abs(loc.posY_mm) < 2));
        octoPose.x = octoPose.y = octoPose.headingDeg = 0;
        prevSeeded = false;
        headingSeeded = false;
        pathMm = turnedDeg = 0;

        if (enc.isDataValid()) {
            rawXOffset  = enc.positions[OctoConfig.CH_X];
            rawX2Offset = enc.positions[OctoConfig.CH_X2];
            rawYOffset  = enc.positions[OctoConfig.CH_Y];
        }
        ypr = imuModule.getIMU().getRobotYawPitchRollAngles();
        hubYawZeroDeg = ypr.getYaw();
        legacy.reset(getImuHeadingRad());
        legacyPose.x = legacyPose.y = legacyPose.headingDeg = 0;

        csv.writeLine(String.format(Locale.US, "# --- lap %d started at %.1f s", trialNum + 1, runtime.seconds()));
    }

    /**
     * Mirror of Odometry.java's WHEEL_IMU path (localization/Odometry.java:
     * calculateRelativeDeltaWheelIMU, linearToArcDelta, rotate, calculateGlobal, and the
     * spike/freeze/phantom fallback in isIMUUnhealthy). Keep this in sync with that file.
     *
     * Exists only because on this test robot all three pods are wired to the OctoQuad, not to
     * hub motor ports, so Odometry's own encoder reads are permanently zero (see class javadoc).
     * Fed pod distances and IMU heading computed from OctoQuad raw counts instead.
     */
    private static final class LegacyOdometry {
        /** Same spike gate as Odometry.isIMUUnhealthy: an 11 deg gap between the IMU and wheel
         *  deltas in one loop is treated as an IMU glitch (shock, 180 deg wrap), not real motion. */
        private static final double SPIKE_RAD = Math.toRadians(11);

        private double x, y, thetaRad;
        private double prevRightMM, prevLeftMM, prevBackMM;
        private double prevImuHeadingRad;
        /** True if the last update() used the wheel heading instead of the IMU (for the LOOP log). */
        boolean usedWheel;

        void reset(double imuHeadingRad) {
            x = 0;
            y = 0;
            thetaRad = 0;
            prevRightMM = 0;
            prevLeftMM = 0;
            prevBackMM = 0;
            prevImuHeadingRad = imuHeadingRad;
        }

        void update(double rightMM, double leftMM, double backMM, double imuHeadingRad) {
            double deltaRight = rightMM - prevRightMM;
            double deltaLeft  = leftMM  - prevLeftMM;
            double deltaBack  = backMM  - prevBackMM;
            prevRightMM = rightMM;
            prevLeftMM  = leftMM;
            prevBackMM  = backMM;

            double wheelDeltaTheta = MathFunctions.angleWrapRad(
                    (deltaLeft - deltaRight) / OctoConfig.TRACK_WIDTH_MM);
            double imuDeltaTheta = MathFunctions.angleWrapRad(imuHeadingRad - prevImuHeadingRad);
            prevImuHeadingRad = imuHeadingRad;

            usedWheel = isUnhealthy(imuDeltaTheta, wheelDeltaTheta);
            double deltaTheta = usedWheel ? wheelDeltaTheta : imuDeltaTheta;

            double deltaX = (deltaLeft + deltaRight) / 2;
            double deltaY = deltaBack - PERP_POD_FWD_MM * deltaTheta;

            // linearToArcDelta: a chord over this loop's rotation instead of the linear
            // approximation, same correction Odometry applies. No-op below ~0.006 deg/loop.
            double relX = deltaX, relY = deltaY;
            if (Math.abs(deltaTheta) >= 1e-4) {
                double forwardRadius = deltaX / deltaTheta;
                double strafeRadius  = deltaY / deltaTheta;
                relX = forwardRadius * Math.sin(deltaTheta) - strafeRadius * (1 - Math.cos(deltaTheta));
                relY = strafeRadius  * Math.sin(deltaTheta) + forwardRadius * (1 - Math.cos(deltaTheta));
            }

            // rotate + calculateGlobal: body-frame delta into field frame by the PREVIOUS
            // heading, then integrate.
            double sinT = Math.sin(thetaRad);
            double cosT = Math.cos(thetaRad);
            x += relX * cosT - relY * sinT;
            y += relY * cosT + relX * sinT;
            thetaRad = MathFunctions.angleWrapRad(thetaRad + deltaTheta);
        }

        /** Same three checks as Odometry.isIMUUnhealthy: spike, freeze, phantom. */
        private boolean isUnhealthy(double imuDeltaTheta, double wheelDeltaTheta) {
            if (Double.isNaN(imuDeltaTheta) || Double.isInfinite(imuDeltaTheta)) return true;
            if (Math.abs(MathFunctions.angleWrapRad(imuDeltaTheta - wheelDeltaTheta)) > SPIKE_RAD) return true;
            if (Math.abs(imuDeltaTheta) < 1e-10 && Math.abs(wheelDeltaTheta) > 1e-3) return true;
            return Math.abs(wheelDeltaTheta) == 0 && Math.abs(imuDeltaTheta) != 0;
        }

        void toPose(OctoConfig.Pose out) {
            out.x = x;
            out.y = y;
            out.headingDeg = Math.toDegrees(thetaRad);
        }
    }
}
