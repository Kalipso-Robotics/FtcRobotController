package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import android.os.Process;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OctoQuadOdo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometryLogger;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometryConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public class OpModeUtilities {
    private final HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private List<LynxModule> allHubs;

    public OpModeUtilities(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        KLog.d("OpModeUtilities", "Constructor called. OpMode: " + opMode.getClass().getName());
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setAllHubs(List<LynxModule> hubs) {
        this.allHubs = hubs;
    }

    public void clearBulkCache() {
        if (allHubs != null) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }

    public static void shutdownExecutorService(ExecutorService executorService) throws InterruptedException {
        KLog.d("ExecutorService_Shutdown", "shutdownExecutorService called");
        // Phase 1: Request graceful shutdown
        executorService.shutdownNow();
        KLog.d("ExecutorService_Shutdown", "Requested shutdown, waiting for tasks to complete...");
        // Phase 2: Wait for tasks to finish gracefully
        boolean terminated = executorService.awaitTermination(1000, TimeUnit.MILLISECONDS);
        KLog.d("ExecutorService_Shutdown", "terminated status " + terminated);

    }

    /**
     * Runs the odometry in a separate thread <p>
     * Executor Service Parameter -> ExecutorService executorService = Executors.newSingleThreadExecutor(); <p>
     * Run shutdown after opMode ends
     * @param executorService
     * @param odometry
     */
    /**
     * Starts odometry from the source OdometryConfig.USE_OCTOQUAD picks (OctoQuad pods or drive-port encoders) and runs it
     * on the executor. Both publish to SharedData.setOdometryWheelIMUPosition, so readers do not change.
     * start != null: a fresh instance at that pose (both singletons are cleared first).
     * start == null: reuse the instance the last OpMode left (auto -> TeleOp pose carry-over), or FieldConfig.CORNER_ZERO_POSE if none.
     * The OctoQuad calibrates its IMU the first time (up to 15 s, keep the robot still) and throws if "octoquad" is not in the config.
     * Pattern source: the Odometry.getInstance + runOdometryExecutorService pairs this replaces.
     */
    public static void startOdometry(OpModeUtilities opModeUtilities, ExecutorService executorService,
                                     DriveTrain driveTrain, IMUModule imuModule, Position start) {
        if (start != null) {
            Odometry.setInstanceNull();
            OctoQuadOdo.setInstanceNull();
        }
        // null = keep the pose from auto if there is one; a fresh instance starts at the corner, not (0,0,0).
        Position fresh = (start == null) ? new Position(FieldConfig.CORNER_ZERO_POSE) : start;
        boolean octo = OdometryConfig.USE_OCTOQUAD;
        if (octo) {
            runOdometryExecutorService(executorService, OctoQuadOdo.getInstance(opModeUtilities, fresh));
        } else {
            runOdometryExecutorService(executorService, Odometry.getInstance(opModeUtilities, driveTrain, imuModule, fresh));
        }
        KLog.d("Robot_Odometry", () -> "event=start source=" + (octo ? "octo" : "wheel") + " start=" + start);
    }

    public static void runOdometryExecutorService(ExecutorService executorService, Odometry odometry) {
        KLog.d("ExecutorService_Run", () -> "runOdometryExecutorService called");
        KLog.d("ExecutorService_Run", () -> "odometry instance: " + odometry);
        KLog.d("ExecutorService_Run", () -> "odometry.getOpModeUtilities(): " + odometry.getOpModeUtilities());

        OdometryLogger odometryLogger = new OdometryLogger("OdometryDataCollector", odometry.getOpModeUtilities());
        KLog.d("ExecutorService_Run", () -> "OdometryLogger created successfully");

        executorService.submit(() -> {
            try {
                KLog.d("ExecutorService_Run", () -> "Executor task started");
                Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
                KLog.d("ExecutorService_Run", () -> "About to check while condition - opModeUtilities=" + odometry.getOpModeUtilities());
                KLog.d("ExecutorService_Run", () -> "opModeUtilities.getOpMode()=" + odometry.getOpModeUtilities().getOpMode().opModeIsActive());
                KLog.d("ExecutorService_Run", () -> "opModeUtilities.getOpMode()=" + odometry.getOpModeUtilities().getOpMode().getClass().getSimpleName());
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        KLog.d("ExecutorService_Run", () -> "Odometry Thread is running. OpMode.isActive: " + odometry.getOpModeUtilities().getOpMode().opModeIsActive());
                        if (odometry.getOpModeUtilities().getOpMode().opModeIsActive()) {
                            KLog.d("ExecutorService_Run", () -> "Odometry OpModeIsActive. Running");
                            odometry.getOpModeUtilities().clearBulkCache();
                            odometry.updateAll();
                            odometryLogger.log(SharedData.getOdometryPositionMap());
                        }
                    } catch (Exception e) {
                        KLog.e("ExecutorService_Run", "Exception in odometry update loop", e);
                        // Continue running despite errors in individual updates
                    }
                }
                KLog.d("ExecutorService_Run", () -> "Odometry thread exiting - interrupted=" +
                    Thread.currentThread().isInterrupted() + ", opModeActive=" +
                    odometry.getOpModeUtilities().getOpMode().opModeIsActive());
            } finally {
                // CRITICAL: This always executes, even if thread is interrupted or exception occurs
                KLog.d("ExecutorService_Run", () -> "Finally. Closing odometry logger");
                odometryLogger.close();
                KLog.d("ExecutorService_Run", () -> "Odometry logger closed successfully");
            }
        });
    }

    /**
     * Runs OctoQuad odometry in a separate thread, the same way as the Odometry overload. <p>
     * No clearBulkCache: the OctoQuad is read over its own I2C, not the hub's bulk cache.
     * No OdometryLogger: it needs a WHEEL history OctoQuadOdo doesn't have, so it skipped every
     * tick and only spammed KLog. Use OctoQuadOdo.getLastTick() for per-tick diagnostics.
     */
    public static void runOdometryExecutorService(ExecutorService executorService, OctoQuadOdo octoQuadOdo) {
        KLog.d("ExecutorService_Run", () -> "runOdometryExecutorService(OctoQuadOdo) called");

        executorService.submit(() -> {
            try {
                Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        if (octoQuadOdo.getOpModeUtilities().getOpMode().opModeIsActive()) {
                            octoQuadOdo.updateAll();
                        }
                    } catch (Exception e) {
                        KLog.e("ExecutorService_Run", "Exception in octoquad update loop", e);
                        // Continue running despite errors in individual updates
                    }
                }
            } finally {
                KLog.d("ExecutorService_Run", () -> "Octoquad executor finished");
            }
        });
    }

    /**
     * Runs SOTM on its own thread, one solve per new odometry sample (blocks on the sample, no sleeps).
     * The thread exits when SOTMConfig.enabled goes false; the caller restarts it (see SOTM.isRunning).
     */
    public static void runSOTMExecutorService(ExecutorService executorService, SOTM sotm) {
        sotm.setRunning(true);
        executorService.submit(() -> {
            Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
            long lastSample = 0;
            try {
                while (!Thread.currentThread().isInterrupted() && SOTMConfig.enabled) {
                    try {
                        if (SharedData.awaitOdometrySampleAfter(lastSample, 50)) {
                            if (sotm.getOpModeUtilities().getOpMode().opModeIsActive()) {
                                sotm.update();
                                lastSample = sotm.getLastSampleNanos();
                            } else {
                                lastSample = System.nanoTime(); // idle: don't spin on old samples
                            }
                        }
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } catch (Exception e) {
                        KLog.e("ExecutorService_Run", "Exception in SOTM update loop", e);
                    }
                }
            } finally {
                sotm.setRunning(false);
            }
        });
    }

    public static void runAprilTagExecutorService(ExecutorService executorService, AprilTagDetectionAction aprilTagDetectionAction) {
        executorService.submit(() -> {
            try {
                KLog.d("ExecutorService", "Executor is running");
                if (aprilTagDetectionAction == null) {
                    KLog.e("ExecutorService_Run", "aprilTagDetectionAction is null, not starting executor");
                    return;
                }


                KLog.d("ExecutorService_Run", () -> "April tag executor task started / running");
                Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);

                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        if (aprilTagDetectionAction.getOpModeUtilities().getOpMode().opModeIsActive()) {
                            aprilTagDetectionAction.updateCheckDone();
                        }
                        // ponytail: fixed nap instead of busy-spinning on the same frame (Limelight <= 90 fps ~ 11 ms);
                        // it used to burn a core the main loop needs. Wait on the frame timestamp if this ever lags.
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } catch (Exception e) {
                        KLog.e("ExecutorService_Run", "Exception in AprilTag update loop", e);
                        // Continue running despite errors in individual updates
                    }
                }
                KLog.d("ExecutorService_Run", () -> "AprilTag thread exiting - interrupted=" +
                        Thread.currentThread().isInterrupted() + ", opModeActive=" +
                        aprilTagDetectionAction.getOpModeUtilities().getOpMode().opModeIsActive());
            } finally {
                // CRITICAL: This always executes, even if thread is interrupted or exception occurs
                KLog.d("ExecutorService_Run", () -> "Closing April Tag Executor Service");
            }
        });
    }
}