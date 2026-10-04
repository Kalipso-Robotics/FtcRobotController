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
 * Optional: press X when passing a second taped mark (CHECK_X/Y_MM from the start mark) to log a
 * CHECK row -- return-to-start can't see a uniform scale error, this can. Trials where the hub IMU
 * froze are flagged legacy_valid=false on the TRIAL row; discard them for the legacy comparison.
 *
 * Filter the CSV for lines starting with "TRIAL" and compare octo_closure_mm vs
 * old_closure_mm across trials (mean, and mm of error per metre driven).
 *
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp
public class CompareOdometryTest extends LinearOpMode {

    // Pod geometry comes from OctoConfig so legacy tracks the SAME point and track width as the
    // board. OctoConfig documents offset = -(pod position) in the board frame (x fwd, y LEFT):
    //   perpendicular pod forward of the TCP = -TCP_OFFSET_MM_X (x axis is shared by both frames)
    //   right parallel pod's distance right of the TCP = -MIRROR * TCP_OFFSET_MM_Y
    /** Perpendicular pod (port 1) forward of the tracking centre; negative = behind. */
    private static double perpPodFwdMM() { return -OctoConfig.TCP_OFFSET_MM_X; }

    /** Right parallel pod (port 0) distance to the RIGHT of the tracking centre. */
    private static double rightPodMM() {
        double mirror = OctoConfig.MIRROR_BOARD_FRAME ? -1.0 : 1.0;
        return -mirror * OctoConfig.TCP_OFFSET_MM_Y;
    }

    /** Hub IMU over-reads rotation by 3.2%: hub yaw / octo heading fit 1.0321 and 1.0317 in the
     *  2026-10-03 laps (pods agree with octo within 0.3%). Re-fit from the logged hub_yaw_deg. */
    private static final double HUB_IMU_HEADING_SCALAR = 1.032;

    private final OctoQuad.LocalizerDataBlock loc =new OctoQuad.LocalizerDataBlock();
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
    /** Hub heading, CW-positive, accumulated from SCALED deltas. Scaling the absolute wrapped yaw
     *  instead injects ~11 deg of error at every +-180 crossing (360 * (1 - 1/scalar)). */
    private double imuAccumRad, prevRawYawRad;
    /** Hub IMU freeze detector: identical yaw/pitch/roll for many loops while the pods keep turning. */
    private double lastYaw, lastPitch, lastRoll, frozenStartPodDeg;
    private int sameLoops;
    private boolean hubFrozen;
    /** Per-lap health, reset in zero(): a lap with any frozen loop is not a valid legacy sample. */
    private int lapLoops, lapFrozenLoops, lapWheelLoops;

    /** Second taped mark for the CHECK row (X button), measured with a tape measure from the
     *  start mark, in the robot's start frame (x fwd, y LEFT). Catches scale error that
     *  return-to-start can't. Set both to the real measurement before running. */
    private static final double CHECK_X_MM = 2438;  // 4 tiles forward
    private static final double CHECK_Y_MM = 0;

    // Raw pod counts are zeroed in SOFTWARE, same trick as OctoTune: setLocalizerPose() resets
    // the board's pose accumulator, not the raw encoder count registers.
    private int rawXOffset, rawX2Offset, rawYOffset;
    /** Zeroed counts from the last VALID read, so CRC-bad reads never reach the CSV. */
    private int lastRawX, lastRawX2, lastRawY;

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
        // This robot's Control Hub stands on edge, logo LEFT, USB UP (2026-10-03: logo-UP/USB-RIGHT
        // read roll -88 deg while flat). If init telemetry pitch/roll aren't ~0, try USB FORWARD/BACKWARD.
        boolean hubImuOk = imuModule.getIMU().initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        csv = new KFileWriter("CompareOdometry", opModeUtilities);
        csv.writeLine("# old = LegacyOdometry (Odometry.java WHEEL_IMU math) on OctoQuad pod counts + hub IMU");
        csv.writeLine("# hub imu mount: logo LEFT, usb UP" + (hubImuOk ? "" : " -- hub imu reinit failed"));
        csv.writeLine(String.format(Locale.US,
                "# legacy geometry (from OctoConfig): track %.2f mm, right pod %.2f mm right of TCP, "
                        + "perp pod %.2f mm fwd of TCP", OctoConfig.TRACK_WIDTH_MM, rightPodMM(), perpPodFwdMM()));
        csv.writeLine("type,t_s,trial,octo_x,octo_y,octo_hdg_deg,old_x,old_y,old_hdg_deg,"
                + "path_mm,turned_deg,octo_closure_mm,octo_hdg_err_deg,old_closure_mm,old_hdg_err_deg,crcOk,loop_ms");
        csv.writeLine("# LOOP rows (every loop): LOOP,t_s,running,octo_x,octo_y,octo_hdg_deg,old_x,old_y,old_hdg_deg,"
                + "pod_hdg_deg,hub_yaw_deg,hub_pitch_deg,hub_roll_deg,old_src,hub_frozen,loop_ms,"
                + "raw_x,raw_x2,raw_y,hub_yaw_raw_deg  (raw_* are zeroed counts: replay any legacy variant offline)");
        csv.writeLine("# TRIAL rows end with hub_frozen_loops,wheel_pct,legacy_valid. "
                + "CHECK rows (X button): CHECK,t_s,trial,octo_x,octo_y,old_x,old_y,err_octo_mm,err_old_mm vs CHECK_X/Y_MM");

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
                updateHubFrozen();
                if (loc.isDataValid() && enc.isDataValid()) {
                    OctoConfig.toRobotFrame(loc, octoPose);
                    if (running) updateOdometer(loc.heading_rad);
                    lastRawX  = enc.positions[OctoConfig.CH_X]  - rawXOffset;
                    lastRawX2 = enc.positions[OctoConfig.CH_X2] - rawX2Offset;
                    lastRawY  = enc.positions[OctoConfig.CH_Y]  - rawYOffset;
                    legacy.update(rightMM(), leftMM(), backMM(), getImuHeadingRad());
                    legacy.toPose(legacyPose);
                    if (running) {
                        lapLoops++;
                        if (hubFrozen) lapFrozenLoops++;
                        if (legacy.usedWheel) lapWheelLoops++;
                    }
                }

                drive.move(gamepad1);

                if (gamepad1.bWasPressed()) {
                    if (running) {
                        csv.writeLine(String.format(Locale.US,
                                "# lap restarted at %.1f s", runtime.seconds()));
                    }
                    zero();
                    running = true;
                } else if (gamepad1.xWasPressed()) {
                    if (running) logCheck();
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
        if (hubFrozen) telemetry.addLine("!!! HUB IMU FROZEN -- legacy is wheel-heading only, lap invalid");
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
        telemetry.addLine("X at 2nd mark = CHECK row (scale). A on start mark = TRIAL.");
    }

    private double podHeadingDeg() {
        return OctoConfig.monitorHeadingDeg(lastRawX, lastRawX2);
    }

    /** One LOOP row per loop, idle or not, so stationary drift is captured too. */
    private void logLoop(double loopMs) {
        csv.writeLine(String.format(Locale.US,
                "LOOP,%.3f,%b,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%s,%.2f,%.2f,%d,%d,%d,%.3f",
                runtime.seconds(), running,
                octoPose.x, octoPose.y, octoPose.headingDeg,
                legacyPose.x, legacyPose.y, legacyPose.headingDeg,
                podHeadingDeg(), ypr.getYaw() - hubYawZeroDeg, ypr.getPitch(), ypr.getRoll(),
                legacy.usedWheel ? "WHEEL" : "IMU", hubFrozen ? 1.0 : 0.0, loopMs,
                lastRawX, lastRawX2, lastRawY, ypr.getYaw()));
    }

    /** One TRIAL summary row: each algorithm's return-to-start error for this lap. */
    private void logTrial(double loopMs) {
        trialNum++;
        double octoClosure = Math.hypot(octoPose.x, octoPose.y);
        double octoHdgErr = Math.abs(MathFunctions.angleWrapDeg(octoPose.headingDeg));
        double oldClosure = Math.hypot(legacyPose.x, legacyPose.y);
        double oldHdgErr = Math.abs(MathFunctions.angleWrapDeg(legacyPose.headingDeg));

        csv.writeLine(String.format(Locale.US,
                "TRIAL,%.3f,%d,%.2f,%.2f,%.4f,%.2f,%.2f,%.4f,%.1f,%.2f,%.2f,%.4f,%.2f,%.4f,%b,%.2f,%d,%.1f,%b",
                runtime.seconds(), trialNum,
                octoPose.x, octoPose.y, octoPose.headingDeg,
                legacyPose.x, legacyPose.y, legacyPose.headingDeg,
                pathMm, turnedDeg,
                octoClosure, octoHdgErr, oldClosure, oldHdgErr,
                loc.crcOk, loopMs,
                lapFrozenLoops, 100.0 * lapWheelLoops / Math.max(1, lapLoops), lapFrozenLoops == 0));
        if (lapFrozenLoops > 0) {
            telemetry.addLine("!!! LEGACY LAP INVALID: hub IMU froze " + lapFrozenLoops + " loops");
        }
        try {
            csv.flush();
        } catch (java.io.IOException e) {
            telemetry.addLine("CSV flush failed: " + e.getMessage());
        }
    }

    /** Pose at the second taped mark; compare to the tape-measured CHECK_X/Y_MM. Mid-lap, no stop. */
    private void logCheck() {
        csv.writeLine(String.format(Locale.US, "CHECK,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                runtime.seconds(), trialNum + 1, octoPose.x, octoPose.y, legacyPose.x, legacyPose.y,
                Math.hypot(octoPose.x - CHECK_X_MM, octoPose.y - CHECK_Y_MM),
                Math.hypot(legacyPose.x - CHECK_X_MM, legacyPose.y - CHECK_Y_MM)));
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

    /** Same convention as Odometry.getIMUHeading() (negated so CW is positive). Stateful: call once
     *  per loop, after ypr is read. */
    private double getImuHeadingRad() {
        double raw = -Math.toRadians(ypr.getYaw());
        imuAccumRad += MathFunctions.angleWrapRad(raw - prevRawYawRad) / HUB_IMU_HEADING_SCALAR;
        prevRawYawRad = raw;
        return imuAccumRad;
    }

    /** True once yaw/pitch/roll have been bit-identical for 60 loops while the pod heading moved
     *  more than 5 deg: a stationary robot can repeat a value, a spinning one cannot. */
    private void updateHubFrozen() {
        double y = ypr.getYaw(), p = ypr.getPitch(), r = ypr.getRoll();
        if (y == lastYaw && p == lastPitch && r == lastRoll) {
            sameLoops++;
        } else {
            sameLoops = 0;
            frozenStartPodDeg = podHeadingDeg();
        }
        lastYaw = y; lastPitch = p; lastRoll = r;
        hubFrozen = sameLoops > 60 && Math.abs(podHeadingDeg() - frozenStartPodDeg) > 5;
    }

    /** Right parallel pod (port 0), forward-positive, same frame as Odometry's right encoder. */
    private double rightMM() {
        return (enc.positions[OctoConfig.CH_X] - rawXOffset) / OctoConfig.COUNTS_PER_MM_X;
    }

    /** Left parallel pod (port 2, monitor channel), forward-positive. */
    private double leftMM() {
        return (enc.positions[OctoConfig.CH_X2] - rawX2Offset) / OctoConfig.COUNTS_PER_MM_X2;
    }

    /** Perpendicular pod (port 1): raw counts go UP moving LEFT (board +y), converted to
     *  +Y-right to match the project frame -- same sign flip as OctoConfig.toRobotFrame. */
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
        lapLoops = lapFrozenLoops = lapWheelLoops = 0;

        if (enc.isDataValid()) {
            rawXOffset  = enc.positions[OctoConfig.CH_X];
            rawX2Offset = enc.positions[OctoConfig.CH_X2];
            rawYOffset  = enc.positions[OctoConfig.CH_Y];
            lastRawX = lastRawX2 = lastRawY = 0;
        }
        ypr = imuModule.getIMU().getRobotYawPitchRollAngles();
        hubYawZeroDeg = ypr.getYaw();
        imuAccumRad = 0;
        prevRawYawRad = -Math.toRadians(ypr.getYaw());
        legacy.reset(0);
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

            // Forward motion of the tracking centre, not the pod midpoint: weight each pod by its
            // distance from the other. Equals (left+right)/2 when the centre is mid-track
            // (rightPod == T/2), which is the original Odometry math.
            double track = OctoConfig.TRACK_WIDTH_MM;
            double rightPod = rightPodMM();
            double deltaX = ((track - rightPod) * deltaRight + rightPod * deltaLeft) / track;
            double deltaY = deltaBack - perpPodFwdMM() * deltaTheta;

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
