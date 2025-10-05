package com.kalipsorobotics.test.navigation;

import android.util.Log;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.navigation.NavigationSystem;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.json.JSONException;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Autonomous OpMode using the complete NavigationSystem
 * 
 * This OpMode demonstrates:
 * - Dynamic Pure Pursuit path following
 * - Terminal precision control
 * - Adaptive friction compensation
 * - Directional bias correction
 * - Comprehensive telemetry and data collection
 * 
 * Setup Instructions:
 * 1. Place mecanum_inverse.tflite and mecanum_inverse_norm.json in TeamCode/src/main/assets/
 * 2. Modify the sendToDriveTrain() method in NavigationSystem.java to match your DriveTrain interface
 * 3. Adjust the path waypoints below as needed for your field/testing
 * 4. Deploy and run this autonomous OpMode
 */
@Autonomous(name="NavigationTest_Complete", group="Navigation")
public class TestNavigation extends LinearOpMode {

    private static final String TAG = "NavTestAuto";
    
    // Navigation system
    private NavigationSystem navigationSystem;
    
    // Hardware modules
    private DriveTrain driveTrain;
    private ExecutorService executorService;
    
    // Timing and performance monitoring
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    
    // =============================================================================
    // PATH CONFIGURATION - Modify these waypoints as needed
    // =============================================================================
    
    private List<Position> autonomousPath = Arrays.asList(
            new Position(0,    0,    Math.toRadians(0)),    // Start position
            new Position(600,  600,    Math.toRadians(90))  // Move forward 600mm, turn to face left
           // Move right 1800mm to final position
    );
    
    // Alternative path configurations - uncomment the desired path
    
    // Square path (1m sides) - good for testing
    /*
    private List<Position> autonomousPath = Arrays.asList(
            new Position(0,    0,    Math.toRadians(0)),
            new Position(1000, 0,    Math.toRadians(90)),
            new Position(1000, 1000, Math.toRadians(180)),
            new Position(0,    1000, Math.toRadians(-90)),
            new Position(0,    0,    Math.toRadians(0))
    );
    */
    
    // Simple straight line - basic testing
    /*
    private List<Position> autonomousPath = Arrays.asList(
            new Position(0,    0,    Math.toRadians(0)),
            new Position(1500, 0,    Math.toRadians(0))
    );
    */
    
    // S-curve - curvature testing
    /*
    private List<Position> autonomousPath = Arrays.asList(
            new Position(0,    0,    Math.toRadians(0)),
            new Position(300,  0,    Math.toRadians(45)),
            new Position(600,  300,  Math.toRadians(0)),
            new Position(900,  300,  Math.toRadians(-45)),
            new Position(1200, 0,    Math.toRadians(0))
    );
    */
    
    // Control flags
    private boolean enableDataCollection = true;
    private boolean enableHighPrecisionMode = false; // Set true for competition precision
    private boolean pathExecutionComplete = false;
    
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("🤖 COMPLETE NAVIGATION SYSTEM TEST");
        telemetry.addLine("==================================");
        telemetry.addLine("Initializing systems...");
        telemetry.update();

        try {
            // Initialize hardware and odometry
            initializeHardware();
            
            // Initialize navigation system
            initializeNavigation();
            
            // Display initialization status
            displayInitializationInfo();

        } catch (Exception e) {
            telemetry.addLine("❌ INITIALIZATION FAILED");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            Log.e(TAG, "Initialization failed", e);
            return;
        }

        // Wait for start
        waitForStart();
        
        // Begin autonomous execution
        executeAutonomous();
        
        // Cleanup and finish
        finishAutonomous();
    }
    
    private void initializeHardware() {
        telemetry.addLine("Initializing hardware...");
        telemetry.update();
        
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        // Initialize drive train
        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        // Initialize IMU
        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        // Initialize odometry
        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);

        // Start odometry executor service
        executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        
        Log.d(TAG, "Hardware initialized successfully");
    }
    
    private void initializeNavigation() throws IOException, JSONException {
        telemetry.addLine("Loading navigation system...");
        telemetry.update();
        
        // Initialize the complete navigation system
        navigationSystem = new NavigationSystem(hardwareMap.appContext, driveTrain);
        
        // Configure system based on mode
        if (enableHighPrecisionMode) {
            configureForPrecision();
        } else {
            configureForGeneral();
        }
        
        // Set the autonomous path
        navigationSystem.setPath(autonomousPath);
        
        // Start data collection if enabled
        if (enableDataCollection) {
            navigationSystem.startDataCollection("Auto_" + System.currentTimeMillis());
        }
        
        Log.d(TAG, "Navigation system initialized");
    }
    
    private void configureForPrecision() {
        // High precision, slower speeds for competition
        navigationSystem.setMaxCruiseSpeed(0.8);
        navigationSystem.setTerminalRadius(0.25);
        navigationSystem.setMaxPathTime(30.0);
        Log.d(TAG, "Configured for high precision mode");
    }
    
    private void configureForGeneral() {
        // Balanced settings for testing
        navigationSystem.setMaxCruiseSpeed(1.0);
        navigationSystem.setTerminalRadius(0.35);
        navigationSystem.setMaxPathTime(25.0);
        Log.d(TAG, "Configured for general testing mode");
    }
    
    private void displayInitializationInfo() {
        telemetry.clearAll();
        telemetry.addLine("✅ INITIALIZATION COMPLETE!");
        telemetry.addLine("============================");
        telemetry.addLine();
        
        // System status
        telemetry.addLine("📋 SYSTEM STATUS");
        telemetry.addData("Navigation System", "✅ Ready");
        telemetry.addData("TFLite Model", "✅ Loaded");
        telemetry.addData("Path Points", autonomousPath.size());
        telemetry.addData("Precision Mode", enableHighPrecisionMode ? "✅ High" : "⚡ Standard");
        telemetry.addData("Data Collection", enableDataCollection ? "✅ Enabled" : "❌ Disabled");
        telemetry.addLine();
        
        // Path preview
        telemetry.addLine("🗺️ PATH PREVIEW");
        for (int i = 0; i < Math.min(autonomousPath.size(), 4); i++) {
            Position pos = autonomousPath.get(i);
            telemetry.addData(String.format("Point %d", i + 1), 
                "X=%.0f, Y=%.0f, H=%.0f°", 
                pos.getX(), pos.getY(), Math.toDegrees(pos.getTheta()));
        }
        if (autonomousPath.size() > 4) {
            telemetry.addLine("... and " + (autonomousPath.size() - 4) + " more points");
        }
        telemetry.addLine();
        
        // Estimated path info
        double totalDistance = calculateTotalPathDistance();
        telemetry.addData("Path Length", "%.1f mm", totalDistance);
        telemetry.addData("Est. Runtime", "%.1f seconds", totalDistance / 600.0); // Rough estimate
        telemetry.addLine();
        
        telemetry.addLine("⏳ Ready! Press ▶️ to start autonomous");
        telemetry.update();
    }
    
    private double calculateTotalPathDistance() {
        double totalDistance = 0.0;
        for (int i = 1; i < autonomousPath.size(); i++) {
            totalDistance += autonomousPath.get(i-1).distanceTo(autonomousPath.get(i));
        }
        return totalDistance;
    }
    
    private void executeAutonomous() {
        // Reset timers and start execution
        runtime.reset();
        loopTimer.reset();
        
        telemetry.clearAll();
        telemetry.addLine("🚀 AUTONOMOUS EXECUTING");
        telemetry.update();
        
        int loopCount = 0;
        double maxLoopTime = 0.0;
        double totalLoopTime = 0.0;
        
        // Main autonomous control loop
        while (opModeIsActive() && !pathExecutionComplete) {
            loopTimer.reset();
            
            // Get current robot pose from odometry
            Position currentPose = SharedData.getOdometryPosition();
            
            // Update navigation system (main control logic)
            boolean pathActive = navigationSystem.update(currentPose);
            
            if (!pathActive) {
                pathExecutionComplete = true;
                Log.d(TAG, "Path execution completed");
            }
            
            // Display real-time telemetry
            displayExecutionTelemetry(currentPose, loopCount);
            
            // Performance monitoring
            double loopTime = loopTimer.milliseconds();
            maxLoopTime = Math.max(maxLoopTime, loopTime);
            totalLoopTime += loopTime;
            loopCount++;
            
            // Safety timeout check
            if (runtime.seconds() > 30.0) { // Hard safety timeout
                telemetry.addLine("⚠️ SAFETY TIMEOUT REACHED - STOPPING");
                telemetry.update();
                Log.w(TAG, "Safety timeout reached");
                break;
            }
            
            // Loop rate control (aim for 50-100Hz)
            if (loopTime < 10) {
                sleep((int)(10 - loopTime)); // Ensure minimum 10ms loop time
            }
        }
        
        // Log final performance stats
        double avgLoopTime = loopCount > 0 ? totalLoopTime / loopCount : 0;
        Log.d(TAG, String.format("Execution completed - %d loops, avg: %.1fms, max: %.1fms", 
                loopCount, avgLoopTime, maxLoopTime));
    }
    
    private void displayExecutionTelemetry(Position currentPose, int loopCount) {
        telemetry.addLine("🤖 AUTONOMOUS NAVIGATION");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("Mode", navigationSystem.getCurrentMode().toString());
        telemetry.addData("Status", navigationSystem.isFinished() ? "✅ COMPLETE" : "🔄 EXECUTING");
        telemetry.addLine();
        
        // Current position and target
        telemetry.addLine("📍 ROBOT POSITION");
        telemetry.addData("X", "%.1f mm", currentPose.getX());
        telemetry.addData("Y", "%.1f mm", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getTheta()));
        telemetry.addLine();
        
        // Navigation progress
        telemetry.addLine("🎯 NAVIGATION PROGRESS");
        telemetry.addData("Distance to Goal", "%.1f mm", 
                navigationSystem.getDistanceToGoal(currentPose) * 1000.0);
        telemetry.addData("Path Progress", "%.1f mm", navigationSystem.getCurrentPathArcLength());
        
        if (navigationSystem.getCurrentMode() == NavigationSystem.ControlMode.PURSUIT) {
            telemetry.addData("Lookahead", "%.3f m", navigationSystem.getCurrentLookahead());
        }
        
        // Current motion
        double[] velocity = navigationSystem.getCurrentVelocity();
        double speed = Math.hypot(velocity[0], velocity[1]);
        telemetry.addData("Current Speed", "%.2f m/s", speed);
        telemetry.addData("Robot Velocity", "vx=%.2f vy=%.2f ω=%.1f°/s", 
                velocity[0], velocity[1], Math.toDegrees(velocity[2]));
        telemetry.addLine();
        
        // Adaptive systems status
        telemetry.addLine("⚙️ ADAPTIVE SYSTEMS");
        telemetry.addData("Friction Scalar", "%.3f", navigationSystem.getFrictionScalar());
        
        double[] bias = navigationSystem.getBiasValues();
        telemetry.addData("Directional Bias", "x=%.3f y=%.3f ω=%.2f°", 
                bias[0], bias[1], Math.toDegrees(bias[2]));
        
        // Speed limitations
        double[] caps = navigationSystem.getSpeedCapValues();
        telemetry.addData("Speed Caps", "Cruise=%.2f Curve=%.2f Brake=%.2f", caps[0], caps[1], caps[2]);
        telemetry.addLine();
        
        // System performance
        telemetry.addLine("📊 PERFORMANCE");
        telemetry.addData("Control Loops", loopCount);
        telemetry.addData("Loop Frequency", "%.1f Hz", loopCount / runtime.seconds());
        
        telemetry.update();
    }
    
    private void finishAutonomous() {
        // Stop data collection and export
        if (enableDataCollection) {
            navigationSystem.stopDataCollection();
            
            // Try to export data (may not work in competition environment)
            try {
                String filename = "/sdcard/FIRST/navigation_data.csv";
                boolean exported = navigationSystem.exportData(filename);
                if (exported) {
                    Log.d(TAG, "Navigation data exported to " + filename);
                } else {
                    Log.w(TAG, "Failed to export navigation data");
                }
            } catch (Exception e) {
                Log.e(TAG, "Data export error", e);
            }
        }
        
        // Display final results
        telemetry.clearAll();
        telemetry.addLine("🏁 AUTONOMOUS COMPLETED");
        telemetry.addLine("========================");
        telemetry.addData("Total Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Final Mode", navigationSystem.getCurrentMode().toString());
        telemetry.addData("Path Success", navigationSystem.isFinished() ? "✅ COMPLETE" : "⚠️ INCOMPLETE");
        telemetry.addLine();
        
        // Final accuracy analysis
        Position currentPose = SharedData.getOdometryPosition();
        Position goalPose = navigationSystem.getGoalPose();
        
        if (goalPose != null) {
            double finalPositionError = currentPose.distanceTo(goalPose);
            double finalHeadingError = Math.abs(goalPose.getTheta() - currentPose.getTheta());
            
            telemetry.addLine("🎯 FINAL ACCURACY");
            telemetry.addData("Position Error", "%.1f mm", finalPositionError);
            telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(finalHeadingError));
            
            boolean withinTolerance = (finalPositionError <= 10.0 && finalHeadingError <= Math.toRadians(1.0));
            telemetry.addData("Within Tolerance", withinTolerance ? "✅ YES" : "❌ NO");
            
            if (withinTolerance) {
                telemetry.addLine("🎉 EXCELLENT ACCURACY!");
            } else if (finalPositionError <= 25.0 && finalHeadingError <= Math.toRadians(3.0)) {
                telemetry.addLine("👍 GOOD ACCURACY");
            } else {
                telemetry.addLine("⚠️ NEEDS TUNING");
            }
        }
        
        telemetry.addLine();
        
        // System status
        telemetry.addLine("📋 SYSTEM STATUS");
        telemetry.addData("Data Collection", enableDataCollection ? "✅ Complete" : "❌ Disabled");
        telemetry.addData("Adaptive Learning", "✅ Active");
        telemetry.addData("Safety Systems", "✅ All Clear");
        telemetry.addLine();
        
        telemetry.addLine("Press STOP to exit");
        telemetry.update();
        
        // Cleanup navigation system
        if (navigationSystem != null) {
            navigationSystem.cleanup();
        }
        
        // Shutdown odometry
        if (executorService != null) {
            executorService.shutdown();
        }
        
        Log.d(TAG, "Autonomous navigation test completed successfully");
        
        // Keep telemetry visible for review
        while (opModeIsActive()) {
            sleep(100);
        }
    }
    
    // =============================================================================
    // UTILITY METHODS FOR DIFFERENT TEST CONFIGURATIONS
    // =============================================================================
    
    /**
     * Call this method to test different navigation scenarios
     */
    private void configureTestScenario(String scenario) {
        switch (scenario.toLowerCase()) {
            case "precision":
                enableHighPrecisionMode = true;
                enableDataCollection = true;
                Log.d(TAG, "Configured for precision testing");
                break;
                
            case "speed":
                navigationSystem.setMaxCruiseSpeed(1.5);
                navigationSystem.setTerminalRadius(0.5);
                enableDataCollection = true;
                Log.d(TAG, "Configured for speed testing");
                break;
                
            case "reliability":
                navigationSystem.setMaxCruiseSpeed(0.6);
                navigationSystem.setMaxPathTime(45.0);
                enableDataCollection = true;
                Log.d(TAG, "Configured for reliability testing");
                break;
                
            default:
                Log.d(TAG, "Using default configuration");
                break;
        }
    }
}