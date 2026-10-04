package org.firstinspires.ftc.teamcode.kalipsorobotics.test.octoquad;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OctoConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OctoQuadOdo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * End-to-end check of the production OctoQuad path, the way any other OpMode would use it:
 * OctoQuadOdo singleton (with a start pose) -> OpModeUtilities.runOdometryExecutorService thread
 * -> SharedData WHEEL_IMU slot. After START the main loop NEVER touches the board; it drives with
 * gamepad1 and reads the pose only from SharedData (plus OctoQuadOdo.getLastTick() for the log).
 *
 * Init (robot still): constructs OctoQuadOdo at START and calibrates the board IMU. Telemetry
 * should already show START. After play: the pose holds at START when still and moves when driven.
 * B = reset pose to START through SharedData (same path as ResetOdometryToPos); flags
 * "RESET LOST" if the executor overwrote it. A (robot back on the start mark) = log closure.
 *
 * CSV (OctoQuadSharedData_*.csv in RobotLogs), one row per loop:
 *   INIT  -- board pose read directly during init (executor not running yet). Any change here
 *            is motion/drift that the first executor tick folds into the pose.
 *   LOOP  -- SharedData pose, the executor's last tick (board pose + pose it wrote, interval,
 *            I2C read time), main loop time, sticks.
 *   RESET / TRIAL -- B and A presses.
 * Offline check: from a RESET (or the first LOOP), pose - ref should equal the board delta rotated
 * by (ref heading - board heading at ref). If it does, the plumbing is exact and any error is the
 * board's own pose; compare to CompareOdometryTest's octo columns.
 *
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp
public class OctoQuadSharedDataTest extends LinearOpMode {

    private static final double START_X_MM = 0;
    private static final double START_Y_MM = 0;
    private static final double START_THETA_DEG = 0;

    private static final double RESET_TOLERANCE_MM = 5;
    private static final double RESET_CHECK_DELAY_MS = 50;
    private static final double STALE_AFTER_S = 1.0;

    private final Position start = new Position(START_X_MM, START_Y_MM, Math.toRadians(START_THETA_DEG));

    private final ElapsedTime runtime = new ElapsedTime();
    private KFileWriter csv;

    @Override
    public void runOpMode() {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        DriveAction drive = new DriveAction(driveTrain);

        csv = new KFileWriter("OctoQuadSharedData", opModeUtilities);
        csv.writeLine("# INIT,t_s,board_valid,board_x,board_y,board_hdg_deg");
        csv.writeLine("# LOOP,t_s,loop_ms,fwd,strafe,turn,shared_x,shared_y,shared_hdg_deg,"
                + "tick_count,ticks_this_loop,tick_interval_ms,tick_read_ms,"
                + "board_x,board_y,board_hdg_deg,tick_x,tick_y,tick_hdg_deg,unhealthy");
        csv.writeLine("# RESET,t_s / TRIAL,t_s,trial,rel_x,rel_y,rel_hdg_deg,closure_mm");
        csv.writeLine(String.format(Locale.US, "# START %.1f,%.1f,%.1f deg", START_X_MM, START_Y_MM, START_THETA_DEG));

        // Robot must be still: the constructor configures the board and calibrates its IMU.
        runtime.reset();
        OctoQuadOdo.setInstanceNull();
        OctoQuadOdo octo = OctoQuadOdo.getInstance(opModeUtilities, start);
        csv.writeLine(String.format(Locale.US, "# calibrated at %.3f s", runtime.seconds()));

        OctoQuad.LocalizerDataBlock initData = new OctoQuad.LocalizerDataBlock();
        OctoConfig.Pose initPose = new OctoConfig.Pose();
        while (opModeInInit()) {
            // Safe: the executor isn't running yet, so this is the only thread on the board.
            octo.getBoard().readLocalizerData(initData);
            boolean valid = initData.isDataValid();
            if (valid) OctoConfig.toRobotFrame(initData, initPose);
            csv.writeLine(String.format(Locale.US, "INIT,%.3f,%b,%.2f,%.2f,%.3f",
                    runtime.seconds(), valid, initPose.x, initPose.y, initPose.headingDeg));

            telemetry.addLine("OCTOQUAD SHARED DATA TEST -- board calibrated, keep still, press START");
            addPose("SharedData pose (want == START)", SharedData.getOdometryWheelIMUPosition());
            telemetry.addData("board pose (want still)", "%.1f / %.1f mm / %.2f deg",
                    initPose.x, initPose.y, initPose.headingDeg);
            telemetry.addData("START", "%.1f / %.1f mm / %.1f deg", START_X_MM, START_Y_MM, START_THETA_DEG);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) { csv.close(); return; }

        ExecutorService exec = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(exec, octo);

        ElapsedTime sinceChange = new ElapsedTime();
        ElapsedTime sinceReset = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime window = new ElapsedTime();
        boolean resetPending = false;
        boolean resetLost = false;
        Position lastPose = SharedData.getOdometryWheelIMUPosition();
        int trials = 0;
        long lastTickCount = 0;
        // Loop-time stats over a 1 s window, shown in telemetry.
        int windowLoops = 0;
        long windowStartTicks = 0;
        double windowMaxLoopMs = 0, windowMaxIntervalMs = 0;
        String loopStats = "--", tickStats = "--";

        try {
            while (opModeIsActive()) {
                double loopMs = loopTimer.milliseconds();
                loopTimer.reset();

                drive.move(gamepad1);

                Position pose = SharedData.getOdometryWheelIMUPosition();
                OctoQuadOdo.Tick tick = octo.getLastTick();
                if (pose.getX() != lastPose.getX() || pose.getY() != lastPose.getY()
                        || pose.getTheta() != lastPose.getTheta()) {
                    sinceChange.reset();
                    lastPose = pose;
                }

                if (gamepad1.bWasPressed()) {
                    SharedData.setOdometryWheelIMUPosition(start);
                    csv.writeLine(String.format(Locale.US, "RESET,%.3f", runtime.seconds()));
                    sinceReset.reset();
                    resetPending = true;
                    resetLost = false;
                } else if (resetPending && sinceReset.milliseconds() > RESET_CHECK_DELAY_MS) {
                    resetPending = false;
                    // Executor is live and we're still, so the pose should have stayed ~START.
                    resetLost = pose.distanceTo(start) > RESET_TOLERANCE_MM;
                    if (resetLost) {
                        KLog.d("octo_shared", () -> "RESET LOST (executor race): " + pose.getPoint());
                        csv.writeLine(String.format(Locale.US, "# RESET LOST at %.3f s", runtime.seconds()));
                    }
                }

                Position rel = relativeToStart(pose);
                if (gamepad1.aWasPressed()) {
                    trials++;
                    double closure = Math.hypot(rel.getX(), rel.getY());
                    csv.writeLine(String.format(Locale.US, "TRIAL,%.3f,%d,%.2f,%.2f,%.3f,%.2f",
                            runtime.seconds(), trials, rel.getX(), rel.getY(),
                            Math.toDegrees(rel.getTheta()), closure));
                    try {
                        csv.flush();
                    } catch (java.io.IOException e) {
                        telemetry.addLine("CSV flush failed: " + e.getMessage());
                    }
                }

                long tickCount = tick == null ? 0 : tick.count;
                csv.writeLine(String.format(Locale.US,
                        "LOOP,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%d",
                        runtime.seconds(), loopMs,
                        -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getTheta()),
                        tickCount, tickCount - lastTickCount,
                        tick == null ? 0 : tick.intervalMs, tick == null ? 0 : tick.readMs,
                        tick == null ? 0 : tick.boardX, tick == null ? 0 : tick.boardY,
                        tick == null ? 0 : Math.toDegrees(tick.boardThetaRad),
                        tick == null ? 0 : tick.pose.getX(), tick == null ? 0 : tick.pose.getY(),
                        tick == null ? 0 : Math.toDegrees(tick.pose.getTheta()),
                        SharedData.getUnhealthyCounter()));
                lastTickCount = tickCount;

                windowLoops++;
                windowMaxLoopMs = Math.max(windowMaxLoopMs, loopMs);
                if (tick != null) windowMaxIntervalMs = Math.max(windowMaxIntervalMs, tick.intervalMs);
                if (window.seconds() >= 1.0) {
                    double s = window.seconds();
                    loopStats = String.format(Locale.US, "avg %.1f ms, max %.1f ms",
                            1000 * s / windowLoops, windowMaxLoopMs);
                    long ticks = tickCount - windowStartTicks;
                    tickStats = ticks == 0 ? "NO TICKS" : String.format(Locale.US,
                            "avg %.1f ms, max %.1f ms, read %.1f ms", 1000 * s / ticks,
                            windowMaxIntervalMs, tick.readMs);
                    window.reset();
                    windowLoops = 0;
                    windowStartTicks = tickCount;
                    windowMaxLoopMs = windowMaxIntervalMs = 0;
                }

                boolean driving = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y)
                        + Math.abs(gamepad1.right_stick_x) > 0.2;

                telemetry.addLine("OCTOQUAD SHARED DATA TEST (reads SharedData only)");
                addPose("SharedData pose", pose);
                telemetry.addData("rel to START (start frame)", "%.1f / %.1f mm / %.2f deg",
                        rel.getX(), rel.getY(), Math.toDegrees(rel.getTheta()));
                Velocity v = SharedData.getOdometryWheelIMUVelocity(); // mm/ms
                telemetry.addData("velocity", "%.0f / %.0f mm/s / %.1f deg/s",
                        v.getX() * 1000, v.getY() * 1000, Math.toDegrees(v.getTheta()) * 1000);
                telemetry.addData("main loop", loopStats);
                telemetry.addData("executor tick", tickStats);
                telemetry.addData("unhealthy counter", SharedData.getUnhealthyCounter());
                telemetry.addData("executor", driving && sinceChange.seconds() > STALE_AFTER_S
                        ? "!!! STALE -- sticks pushed but pose frozen" : "ok");
                if (resetLost) telemetry.addLine("!!! RESET LOST (executor race)");
                telemetry.addData("trials logged", trials);
                telemetry.addLine("B = reset to START, A on the start mark = log closure");
                telemetry.update();
            }
        } finally {
            exec.shutdownNow();
            csv.close();
        }
    }

    private void addPose(String caption, Position p) {
        telemetry.addData(caption, "%.1f / %.1f mm / %.2f deg",
                p.getX(), p.getY(), Math.toDegrees(p.getTheta()));
    }

    /** Pose minus START, with x/y rotated into the start frame; heading wrapped. */
    private Position relativeToStart(Position p) {
        double dx = p.getX() - start.getX();
        double dy = p.getY() - start.getY();
        double c = Math.cos(-start.getTheta()), s = Math.sin(-start.getTheta());
        return new Position(dx * c - dy * s, dy * c + dx * s,
                MathFunctions.angleWrapRad(p.getTheta() - start.getTheta()));
    }
}
