package com.kalipsorobotics.utilities;

import android.os.Process;
import android.util.Log;

import com.kalipsorobotics.localization.Odometry;
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
        executorService.shutdownNow();
        executorService.awaitTermination(1, TimeUnit.SECONDS);
    }

    /**
     * Runs the odometry in a separate thread <p>
     * Executor Service Parameter -> ExecutorService executorService = Executors.newSingleThreadExecutor(); <p>
     * Run shutdown after opMode ends
     * @param executorService
     * @param odometry
     */
    public static void runOdometryExecutorService(ExecutorService executorService, Odometry odometry) {
        try {
            executorService.submit(() -> {
                Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
                while (!Thread.currentThread().isInterrupted()) {
                    Log.d("ExcecutorService", "running");
                    odometry.update();
                    Log.d("ExcecutorService", "running after update");

                }

            });
        } catch (RuntimeException e) {
            Log.e("ExecutorService", "A Runtime Exception occurred",e);
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            Log.e("ExecutorService", "An exception occurred",e);
            Thread.currentThread().interrupt();
        }
    }
}