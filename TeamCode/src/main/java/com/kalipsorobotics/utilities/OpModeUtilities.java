package com.kalipsorobotics.utilities;

import android.os.Process;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.localization.OdometryLogger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public class OpModeUtilities {
    private final HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

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
    public static void runOdometryExecutorService(ExecutorService executorService, Odometry odometry) {
        KLog.d("ExecutorService_Run", "runOdometryExecutorService called");
        KLog.d("ExecutorService_Run", "odometry instance: " + odometry);
        KLog.d("ExecutorService_Run", "odometry.getOpModeUtilities(): " + odometry.getOpModeUtilities());

        OdometryLogger odometryLogger = new OdometryLogger("OdometryDataCollector", odometry.getOpModeUtilities());
        KLog.d("ExecutorService_Run", "OdometryLogger created successfully");

        executorService.submit(() -> {
            try {
                KLog.d("ExecutorService_Run", "Executor task started");
                Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
                KLog.d("ExecutorService_Run", "About to check while condition - opModeUtilities=" + odometry.getOpModeUtilities());
                KLog.d("ExecutorService_Run", "opModeUtilities.getOpMode()=" + odometry.getOpModeUtilities().getOpMode().opModeIsActive());
                KLog.d("ExecutorService_Run", "opModeUtilities.getOpMode()=" + odometry.getOpModeUtilities().getOpMode().getClass().getSimpleName());
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        KLog.d("ExecutorService_Run", "Thread is running. OpMode.isActive: " + odometry.getOpModeUtilities().getOpMode().opModeIsActive());
                        if (odometry.getOpModeUtilities().getOpMode().opModeIsActive()) {
                            KLog.d("ExecutorService_Run", "OpModeIsActive. Running");
                            odometry.updateAll();
                            odometryLogger.log(SharedData.getOdometryPositionMap());
                        }
                    } catch (Exception e) {
                        KLog.e("ExecutorService_Run", "Exception in odometry update loop", e);
                        // Continue running despite errors in individual updates
                    }
                }
                KLog.d("ExecutorService_Run", "Odometry thread exiting - interrupted=" +
                    Thread.currentThread().isInterrupted() + ", opModeActive=" +
                    odometry.getOpModeUtilities().getOpMode().opModeIsActive());
            } finally {
                // CRITICAL: This always executes, even if thread is interrupted or exception occurs
                KLog.d("ExecutorService_Run", "Finally. Closing odometry logger");
                odometryLogger.close();
                KLog.d("ExecutorService_Run", "Odometry logger closed successfully");
            }
        });
    }
}