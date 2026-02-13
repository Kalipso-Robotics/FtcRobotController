package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import android.content.Context;
import android.util.Log;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.tensorflow.lite.Interpreter;

import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.nio.MappedByteBuffer;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Complete FTC Navigation System - All Classes Merged
 * Implements Dynamic Pure Pursuit + Terminal Control with Adaptive Systems
 * 
 * Features:
 * - TensorFlow Lite inverse model integration
 * - Catmull-Rom spline path sampling
 * - Dynamic Pure Pursuit controller
 * - Terminal pose precision control
 * - Adaptive friction compensation
 * - Directional bias correction
 * - Multi-layer speed limiting
 * - Comprehensive data collection
 * - DriveTrain integration with safety features
 * 
 * Usage:
 * 1. Create instance: NavigationSystem nav = new NavigationSystem(context, driveTrain);
 * 2. Set path: nav.setPath(waypoints);
 * 3. Main loop: nav.update(currentPose);
 * 4. Check status: nav.isFinished();
 */
public class NavigationSystem {
    private static final String TAG = "NavigationSystem";
    
    // =============================================================================
    // MAIN NAVIGATION CONTROLLER
    // =============================================================================
    
    // Control modes
    public enum ControlMode {
        PURSUIT,     // Following path with dynamic pure pursuit
        TERMINAL,    // Final pose precision control
        FINISHED     // Task completed
    }
    
    // Core components
    private final InverseModelDriver inverseModel;
    private final PathSampler pathSampler;
    private final PursuitController pursuitController;
    private final TerminalPoseController terminalController;
    private final FrictionAdaptor frictionAdaptor;
    private final BiasAdaptor biasAdaptor;
    private final SpeedCaps speedCaps;
    private final DriveTrainIntegration driveTrainIntegration;
    private final DataCollector dataCollector;
    
    // State variables
    private ControlMode currentMode = ControlMode.PURSUIT;
    private Position goalPose;
    private double terminalRadius = 0.10; // meters - switch to terminal mode
    private long lastUpdateTime = -1;
    private Position lastPose = null;
    
    // Control parameters
    private double maxCruiseSpeed = 1.2; // m/s
    private final double maxBrakingAccel = 2.5; // m/s²
    
    // Telemetry and status
    private double currentLookahead = 0.0;
    private double[] currentVelocity = new double[3];
    private double[] speedCapValues = new double[3];
    private double currentPathArcLength = 0.0;
    private boolean isInitialized = false;
    
    // Timeout protection
    private long pathStartTime = -1;
    private double maxPathTimeSeconds = 30.0;
    
    /**
     * Initialize the complete navigation system
     */
    public NavigationSystem(Context context, DriveTrain driveTrain) throws IOException, JSONException {
        // Initialize all components
        inverseModel = new InverseModelDriver(context, "mecanum_inverse.tflite", "mecanum_inverse_norm.json");
        pathSampler = new PathSampler();
        driveTrainIntegration = new DriveTrainIntegration(driveTrain);
        dataCollector = new DataCollector();
        
        // Initialize controllers with tuned parameters
        pursuitController = new PursuitController();
        pursuitController.setLookaheadParameters(0.20, 0.80, 1.0);
        pursuitController.setControlGains(2.9, 2.9);
        
        terminalController = new TerminalPoseController();
        terminalController.setResponseTimes(0.30, 0.25);
        terminalController.setTerminalLimits(0.35, 4.0);
        terminalController.setTolerances(0.010, Math.toRadians(1.0), 100);
        
        // Initialize adaptive systems
        frictionAdaptor = new FrictionAdaptor();
        frictionAdaptor.setParameters(0.120, 0.8, 0.95);
        frictionAdaptor.setSpeedScalarLimits(0.6, 1.0);
        
        biasAdaptor = new BiasAdaptor();
        biasAdaptor.setIntegrationRates(0.02, 0.05);
        biasAdaptor.setBiasLimits(0.05, 0.4);
        biasAdaptor.setAdaptationConditions(0.1, 0.5, 0.5);
        
        // Initialize speed caps
        speedCaps = new SpeedCaps();
        speedCaps.setMaxCruiseSpeed(maxCruiseSpeed);
        speedCaps.setBrakingAcceleration(maxBrakingAccel);
        speedCaps.setCurvatureGain(0.8);
        
        isInitialized = true;
        KLog.d(TAG, "NavigationSystem initialized successfully");
    }
    
    /**
     * Set the path to follow
     */
    public void setPath(List<Position> waypoints) {
        if (!isInitialized || waypoints == null || waypoints.isEmpty()) {
            KLog.e(TAG, "Cannot set path - not initialized or invalid waypoints");
            return;
        }
        
        try {
            pathSampler.buildPath(waypoints);
            goalPose = new Position(waypoints.get(waypoints.size() - 1));
            
            // Reset state
            currentMode = ControlMode.PURSUIT;
            lastUpdateTime = -1;
            lastPose = null;
            pathStartTime = System.currentTimeMillis();
            
            // Reset adaptive components
            frictionAdaptor.reset();
            biasAdaptor.reset();
            terminalController.reset();
            
            KLog.d(TAG, String.format("Path set with %d waypoints", waypoints.size()));
            
        } catch (Exception e) {
            KLog.e(TAG, "Failed to set path", e);
            currentMode = ControlMode.FINISHED;
        }
    }
    
    /**
     * Main control loop update - call continuously at 50-100Hz
     */
    public boolean update(Position currentPose) {
        if (!isInitialized || pathSampler.isEmpty() || goalPose == null) {
            stopRobot();
            return false;
        }
        
        // Check for timeout
        if (pathStartTime > 0) {
            double elapsedTime = (System.currentTimeMillis() - pathStartTime) / 1000.0;
            if (elapsedTime > maxPathTimeSeconds) {
                Log.w(TAG, "Path following timeout reached");
                currentMode = ControlMode.FINISHED;
                stopRobot();
                return false;
            }
        }
        
        // Initialize timing
        long currentTime = System.currentTimeMillis();
        if (lastUpdateTime < 0) {
            lastUpdateTime = currentTime;
            lastPose = new Position(currentPose);
            return true;
        }
        
        long deltaTimeMs = currentTime - lastUpdateTime;
        double deltaTimeS = deltaTimeMs / 1000.0;
        
        // Skip if called too frequently or too slowly
        if (deltaTimeS < 0.005 || deltaTimeS > 0.1) {
            return currentMode != ControlMode.FINISHED;
        }
        
        // Compute current speed
        double currentSpeed = computeCurrentSpeed(currentPose, deltaTimeS);
        
        // Determine control mode
        updateControlMode(currentPose);
        
        if (currentMode == ControlMode.FINISHED) {
            stopRobot();
            return false;
        }
        
        // Compute velocity command
        double[] velocity = computeVelocityCommand(currentPose, currentSpeed);
        
        // Apply all modifications
        applyControlModifications(currentPose, velocity, deltaTimeS);
        
        // Send to motors
        sendMotorCommands(velocity);
        
        // Update state
        updateState(currentPose, velocity, currentTime);
        
        return true;
    }
    
    private double computeCurrentSpeed(Position currentPose, double deltaTimeS) {
        if (lastPose == null) return 0.0;
        
        double deltaX = (currentPose.getX() - lastPose.getX()) / 1000.0;
        double deltaY = (currentPose.getY() - lastPose.getY()) / 1000.0;
        return Math.hypot(deltaX, deltaY) / deltaTimeS;
    }
    
    private void updateControlMode(Position currentPose) {
        double distanceToGoal = currentPose.distanceTo(goalPose) / 1000.0;
        
        if (currentMode == ControlMode.PURSUIT && distanceToGoal <= terminalRadius) {
            currentMode = ControlMode.TERMINAL;
            KLog.d(TAG, "Switching to terminal control mode");
        }
        
        if (currentMode == ControlMode.TERMINAL && 
            terminalController.isFinished(goalPose, currentPose)) {
            currentMode = ControlMode.FINISHED;
            KLog.d(TAG, "Path following completed!");
        }
    }
    
    private double[] computeVelocityCommand(Position currentPose, double currentSpeed) {
        double[] velocity = new double[3];
        
        if (currentMode == ControlMode.PURSUIT) {
            PursuitController.VelocityCommand cmd = pursuitController.computePursuitVelocity(
                    pathSampler, currentPose.getX(), currentPose.getY(), 
                    currentPose.getTheta(), currentSpeed);
            
            velocity[0] = cmd.vx;
            velocity[1] = cmd.vy;
            velocity[2] = cmd.omega;
            
            // Update telemetry
            int nearestIndex = pathSampler.nearestIndexTo(currentPose.getX(), currentPose.getY());
            currentPathArcLength = pathSampler.getArcLengthAt(nearestIndex);
            currentLookahead = Math.max(0.20, Math.min(0.80, 0.20 + currentSpeed));
            
        } else if (currentMode == ControlMode.TERMINAL) {
            TerminalPoseController.VelocityCommand cmd = 
                    terminalController.computeTerminalVelocity(goalPose, currentPose);
            
            velocity[0] = cmd.vx;
            velocity[1] = cmd.vy;
            velocity[2] = cmd.omega;
        }
        
        return velocity;
    }
    
    private void applyControlModifications(Position currentPose, double[] velocity, double deltaTimeS) {
        // Get path curvature
        double pathCurvature = getCurrentPathCurvature(currentPose);
        
        // Apply speed caps
        speedCaps.applyCaps(velocity, currentPose, goalPose, pathCurvature);
        speedCapValues = speedCaps.getCapValues(currentPose, goalPose, pathCurvature);
        
        // Apply bias adaptation
        if (currentMode == ControlMode.TERMINAL) {
            double[] robotError = computeRobotFrameError(goalPose, currentPose);
            double headingError = wrapAngle(goalPose.getTheta() - currentPose.getTheta());
            
            biasAdaptor.update(robotError[0], robotError[1], headingError, 
                             velocity[2], pathCurvature, deltaTimeS);
        }
        
        biasAdaptor.applyBias(velocity);
        
        // Apply friction adaptation
        if (lastPose != null) {
            double measuredDeltaX = currentPose.getX() - lastPose.getX();
            double measuredDeltaY = currentPose.getY() - lastPose.getY();
            
            frictionAdaptor.update(currentVelocity[0], currentVelocity[1], 
                                 measuredDeltaX, measuredDeltaY, (long)(deltaTimeS * 1000));
        }
        
        frictionAdaptor.scaleVelocity(velocity);
    }
    
    private void sendMotorCommands(double[] velocity) {
        try {
            float[] motorPowers = inverseModel.getMotorPowers(velocity[0], velocity[1], velocity[2]);
            driveTrainIntegration.setMotorPowers(motorPowers);
        } catch (Exception e) {
            KLog.e(TAG, "Failed to send motor commands", e);
            stopRobot();
        }
    }
    
    private void stopRobot() {
        driveTrainIntegration.emergencyStop();
        currentVelocity = new double[]{0, 0, 0};
    }
    
    private double getCurrentPathCurvature(Position currentPose) {
        if (pathSampler.isEmpty()) return 0.0;
        
        int nearestIndex = pathSampler.nearestIndexTo(currentPose.getX(), currentPose.getY());
        double arcLength = pathSampler.getArcLengthAt(nearestIndex);
        return pathSampler.curvatureAt(arcLength);
    }
    
    private double[] computeRobotFrameError(Position goal, Position current) {
        double fieldErrorX = (goal.getX() - current.getX()) / 1000.0;
        double fieldErrorY = (goal.getY() - current.getY()) / 1000.0;
        
        double cosHeading = Math.cos(current.getTheta());
        double sinHeading = Math.sin(current.getTheta());
        
        double robotErrorX = fieldErrorX * cosHeading + fieldErrorY * sinHeading;
        double robotErrorY = -fieldErrorX * sinHeading + fieldErrorY * cosHeading;
        
        return new double[]{robotErrorX, robotErrorY};
    }
    
    private void updateState(Position currentPose, double[] velocity, long currentTime) {
        currentVelocity = velocity.clone();
        lastUpdateTime = currentTime;
        lastPose = new Position(currentPose);
    }
    
    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    // Public getters for telemetry
    public ControlMode getCurrentMode() { return currentMode; }
    public boolean isFinished() { return currentMode == ControlMode.FINISHED; }
    public double getCurrentLookahead() { return currentLookahead; }
    public double[] getCurrentVelocity() { return currentVelocity.clone(); }
    public double[] getSpeedCapValues() { return speedCapValues.clone(); }
    public double getFrictionScalar() { return frictionAdaptor.getSpeedScalar(); }
    public double[] getBiasValues() { return biasAdaptor.getBias(); }
    public double getCurrentPathArcLength() { return currentPathArcLength; }
    public double getDistanceToGoal(Position currentPose) { 
        return goalPose != null ? currentPose.distanceTo(goalPose) / 1000.0 : 0.0; 
    }
    public Position getGoalPose() { return goalPose; }
    
    // Configuration methods
    public void setTerminalRadius(double radius) { this.terminalRadius = radius; }
    public void setMaxPathTime(double seconds) { this.maxPathTimeSeconds = seconds; }
    public void setMaxCruiseSpeed(double speed) { 
        this.maxCruiseSpeed = speed;
        if (speedCaps != null) speedCaps.setMaxCruiseSpeed(speed);
    }
    
    // Data collection
    public void startDataCollection(String sessionName) { dataCollector.startCollection(sessionName); }
    public void stopDataCollection() { dataCollector.stopCollection(); }
    public boolean exportData(String filename) { return dataCollector.exportToCSV(filename); }
    
    public void cleanup() {
        if (inverseModel != null) inverseModel.close();
        if (driveTrainIntegration != null) driveTrainIntegration.emergencyStop();
        KLog.d(TAG, "NavigationSystem cleaned up");
    }
    
    // =============================================================================
    // INVERSE MODEL DRIVER
    // =============================================================================
    
    private static class InverseModelDriver {
        private static final String TAG = "InverseModelDriver";
        
        private Interpreter interpreter;
        private final float[] inputMeans = new float[3];
        private final float[] inputStds = new float[3];
        private final float[][] inputArray = new float[1][3];
        private final float[][] outputArray = new float[1][4];
        
        public InverseModelDriver(Context context, String modelPath, String normPath) 
                throws IOException, JSONException {
            loadNormalization(context, normPath);
            loadModel(context, modelPath);
        }
        
        private void loadNormalization(Context context, String normPath) throws IOException, JSONException {
            InputStream is = context.getAssets().open(normPath);
            byte[] buffer = new byte[is.available()];
            is.read(buffer);
            is.close();
            
            String json = new String(buffer, StandardCharsets.UTF_8);
            JSONObject obj = new JSONObject(json);
            
            JSONArray means = obj.getJSONArray("x_mean");
            JSONArray stds = obj.getJSONArray("x_std");
            
            for (int i = 0; i < 3; i++) {
                inputMeans[i] = (float) means.getDouble(i);
                inputStds[i] = (float) stds.getDouble(i);
            }
            
            KLog.d(TAG, "Loaded normalization parameters");
        }
        
        private void loadModel(Context context, String modelPath) throws IOException {
            InputStream is = context.getAssets().open(modelPath);
            byte[] buffer = new byte[is.available()];
            is.read(buffer);
            is.close();
            
            java.nio.ByteBuffer modelBuffer = java.nio.ByteBuffer.allocateDirect(buffer.length);
            modelBuffer.put(buffer);
            
            interpreter = new Interpreter(modelBuffer);
            KLog.d(TAG, "Loaded TFLite model");
        }
        
        public float[] getMotorPowers(double vx, double vy, double omega) {
            try {
                // Normalize input
                float[] normalized = new float[3];
                normalized[0] = (float) ((vx - inputMeans[0]) / inputStds[0]);
                normalized[1] = (float) ((vy - inputMeans[1]) / inputStds[1]);
                normalized[2] = (float) ((omega - inputMeans[2]) / inputStds[2]);
                
                // Run inference
                inputArray[0] = normalized;
                interpreter.run(inputArray, outputArray);
                float[] outputs = outputArray[0].clone();
                
                // Clamp outputs
                for (int i = 0; i < outputs.length; i++) {
                    outputs[i] = Math.max(-1.0f, Math.min(1.0f, outputs[i]));
                }
                
                return outputs;
                
            } catch (Exception e) {
                KLog.e(TAG, "TFLite failed, using fallback", e);
                return simpleMecanumKinematics(vx, vy, omega);
            }
        }
        
        private float[] simpleMecanumKinematics(double vx, double vy, double omega) {
            float fl = (float) (vx + vy + omega);
            float fr = (float) (vx - vy - omega);
            float bl = (float) (vx - vy + omega);
            float br = (float) (vx + vy - omega);
            
            float maxPower = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), 
                                     Math.max(Math.abs(bl), Math.abs(br)));
            if (maxPower > 1.0f) {
                fl /= maxPower;
                fr /= maxPower;
                bl /= maxPower;
                br /= maxPower;
            }
            
            return new float[]{fl, fr, bl, br};
        }
        
        public void close() {
            if (interpreter != null) interpreter.close();
        }
    }
    
    // =============================================================================
    // PATH SAMPLER
    // =============================================================================
    
    private static class PathSampler {
        private static final double SAMPLE_DISTANCE_MM = 10.0;
        
        private final List<PathPoint> sampledPoints = new ArrayList<>();
        private double totalPathLength = 0.0;
        private int lastNearestIndex = 0;
        
        public static class PathPoint {
            public double x, y, heading;
            public double tangentX, tangentY;
            public double curvature;
            public double arcLength;
            
            public PathPoint(double x, double y, double heading, double tangentX, double tangentY, 
                           double curvature, double arcLength) {
                this.x = x; this.y = y; this.heading = heading;
                this.tangentX = tangentX; this.tangentY = tangentY;
                this.curvature = curvature; this.arcLength = arcLength;
            }
        }
        
        public void buildPath(List<Position> waypoints) {
            if (waypoints.size() < 2) {
                throw new IllegalArgumentException("Need at least 2 waypoints");
            }
            
            sampledPoints.clear();
            lastNearestIndex = 0;
            
            // Extend waypoints for Catmull-Rom
            List<Position> extended = new ArrayList<>();
            extended.add(waypoints.get(0));
            extended.addAll(waypoints);
            extended.add(waypoints.get(waypoints.size() - 1));
            
            double cumulativeLength = 0.0;
            
            // Sample each segment
            for (int i = 1; i < extended.size() - 2; i++) {
                Position p0 = extended.get(i - 1);
                Position p1 = extended.get(i);
                Position p2 = extended.get(i + 1);
                Position p3 = extended.get(i + 2);
                
                double segmentLength = p1.distanceTo(p2);
                int numSamples = Math.max(2, (int) Math.ceil(segmentLength / SAMPLE_DISTANCE_MM));
                
                for (int j = 0; j < numSamples; j++) {
                    if (i > 1 && j == 0) continue;
                    
                    double t = (double) j / (numSamples - 1);
                    
                    double[] pos = catmullRomInterpolate(p0, p1, p2, p3, t);
                    double[] tangent = catmullRomTangent(p0, p1, p2, p3, t);
                    
                    double tangentMag = Math.hypot(tangent[0], tangent[1]);
                    if (tangentMag > 1e-6) {
                        tangent[0] /= tangentMag;
                        tangent[1] /= tangentMag;
                    }
                    
                    double heading = Math.atan2(tangent[1], tangent[0]);
                    
                    if (sampledPoints.size() > 0) {
                        PathPoint lastPoint = sampledPoints.get(sampledPoints.size() - 1);
                        cumulativeLength += Math.hypot(pos[0] - lastPoint.x, pos[1] - lastPoint.y);
                    }
                    
                    PathPoint point = new PathPoint(pos[0], pos[1], heading, tangent[0], tangent[1], 
                                                  0.0, cumulativeLength);
                    sampledPoints.add(point);
                }
            }
            
            computeCurvature();
            totalPathLength = cumulativeLength;
        }
        
        private double[] catmullRomInterpolate(Position p0, Position p1, Position p2, Position p3, double t) {
            double t2 = t * t;
            double t3 = t2 * t;
            
            double x = 0.5 * ((2 * p1.getX()) +
                             (-p0.getX() + p2.getX()) * t +
                             (2 * p0.getX() - 5 * p1.getX() + 4 * p2.getX() - p3.getX()) * t2 +
                             (-p0.getX() + 3 * p1.getX() - 3 * p2.getX() + p3.getX()) * t3);
            
            double y = 0.5 * ((2 * p1.getY()) +
                             (-p0.getY() + p2.getY()) * t +
                             (2 * p0.getY() - 5 * p1.getY() + 4 * p2.getY() - p3.getY()) * t2 +
                             (-p0.getY() + 3 * p1.getY() - 3 * p2.getY() + p3.getY()) * t3);
            
            return new double[]{x, y};
        }
        
        private double[] catmullRomTangent(Position p0, Position p1, Position p2, Position p3, double t) {
            double t2 = t * t;
            
            double dx = 0.5 * ((-p0.getX() + p2.getX()) +
                              (2 * p0.getX() - 5 * p1.getX() + 4 * p2.getX() - p3.getX()) * 2 * t +
                              (-p0.getX() + 3 * p1.getX() - 3 * p2.getX() + p3.getX()) * 3 * t2);
            
            double dy = 0.5 * ((-p0.getY() + p2.getY()) +
                              (2 * p0.getY() - 5 * p1.getY() + 4 * p2.getY() - p3.getY()) * 2 * t +
                              (-p0.getY() + 3 * p1.getY() - 3 * p2.getY() + p3.getY()) * 3 * t2);
            
            return new double[]{dx, dy};
        }
        
        private void computeCurvature() {
            for (int i = 1; i < sampledPoints.size() - 1; i++) {
                PathPoint prev = sampledPoints.get(i - 1);
                PathPoint curr = sampledPoints.get(i);
                PathPoint next = sampledPoints.get(i + 1);
                
                double dx1 = curr.x - prev.x;
                double dy1 = curr.y - prev.y;
                double dx2 = next.x - curr.x;
                double dy2 = next.y - curr.y;
                
                double cross = dx1 * dy2 - dy1 * dx2;
                double ds1 = Math.hypot(dx1, dy1);
                double ds2 = Math.hypot(dx2, dy2);
                
                if (ds1 > 1e-6 && ds2 > 1e-6) {
                    curr.curvature = Math.abs(cross) / (ds1 * ds2 * (ds1 + ds2) / 2.0) / 1000.0;
                }
            }
            
            if (sampledPoints.size() >= 2) {
                sampledPoints.get(0).curvature = sampledPoints.get(1).curvature;
                sampledPoints.get(sampledPoints.size() - 1).curvature = 
                    sampledPoints.get(sampledPoints.size() - 2).curvature;
            }
        }
        
        public int nearestIndexTo(double x, double y) {
            if (sampledPoints.isEmpty()) return -1;
            
            int bestIndex = lastNearestIndex;
            double bestDist = distanceSquaredToPoint(x, y, bestIndex);
            
            // Search forward
            for (int i = lastNearestIndex + 1; i < sampledPoints.size(); i++) {
                double dist = distanceSquaredToPoint(x, y, i);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIndex = i;
                } else {
                    break;
                }
            }
            
            // Search backward
            for (int i = lastNearestIndex - 1; i >= 0; i--) {
                double dist = distanceSquaredToPoint(x, y, i);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIndex = i;
                } else {
                    break;
                }
            }
            
            lastNearestIndex = bestIndex;
            return bestIndex;
        }
        
        private double distanceSquaredToPoint(double x, double y, int index) {
            if (index < 0 || index >= sampledPoints.size()) return Double.MAX_VALUE;
            PathPoint p = sampledPoints.get(index);
            double dx = x - p.x;
            double dy = y - p.y;
            return dx * dx + dy * dy;
        }
        
        public PathPoint pointAtArc(double targetArcLength) {
            if (sampledPoints.isEmpty()) return null;
            
            targetArcLength = Math.max(0, Math.min(targetArcLength, totalPathLength));
            
            for (int i = 0; i < sampledPoints.size() - 1; i++) {
                PathPoint p1 = sampledPoints.get(i);
                PathPoint p2 = sampledPoints.get(i + 1);
                
                if (targetArcLength >= p1.arcLength && targetArcLength <= p2.arcLength) {
                    if (Math.abs(p2.arcLength - p1.arcLength) < 1e-6) {
                        return p1;
                    }
                    
                    double t = (targetArcLength - p1.arcLength) / (p2.arcLength - p1.arcLength);
                    
                    return new PathPoint(
                        p1.x + t * (p2.x - p1.x),
                        p1.y + t * (p2.y - p1.y),
                        p1.heading + t * wrapAngleDiff(p2.heading - p1.heading),
                        p1.tangentX + t * (p2.tangentX - p1.tangentX),
                        p1.tangentY + t * (p2.tangentY - p1.tangentY),
                        p1.curvature + t * (p2.curvature - p1.curvature),
                        targetArcLength
                    );
                }
            }
            
            return sampledPoints.get(sampledPoints.size() - 1);
        }
        
        public double curvatureAt(double arcLength) {
            PathPoint p = pointAtArc(arcLength);
            return p != null ? p.curvature : 0.0;
        }
        
        public double getArcLengthAt(int index) {
            if (index >= 0 && index < sampledPoints.size()) {
                return sampledPoints.get(index).arcLength;
            }
            return 0.0;
        }
        
        public boolean isEmpty() { return sampledPoints.isEmpty(); }
        
        private double wrapAngleDiff(double angleDiff) {
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
            return angleDiff;
        }
    }
    
    // =============================================================================
    // PURSUIT CONTROLLER
    // =============================================================================
    
    private static class PursuitController {
        
        private double minLookahead = 0.20; // meters
        private double maxLookahead = 0.80; // meters
        private double lookaheadGain = 1.0; // seconds
        private double translationGain = 2.9; // 1/0.35s
        private double headingGain = 2.9; // 1/0.35s
        
        public static class VelocityCommand {
            public double vx, vy, omega; // m/s, m/s, rad/s
            
            public VelocityCommand(double vx, double vy, double omega) {
                this.vx = vx; this.vy = vy; this.omega = omega;
            }
        }
        
        public VelocityCommand computePursuitVelocity(PathSampler pathSampler, double robotX, double robotY, 
                                                     double robotHeading, double currentSpeed) {
            
            // Convert robot position from mm to meters
            double robotXM = robotX / 1000.0;
            double robotYM = robotY / 1000.0;
            
            // Find nearest point on path
            int nearestIndex = pathSampler.nearestIndexTo(robotX, robotY);
            if (nearestIndex < 0) {
                return new VelocityCommand(0, 0, 0);
            }
            
            double nearestArcLength = pathSampler.getArcLengthAt(nearestIndex) / 1000.0; // Convert to meters
            
            // Calculate dynamic lookahead distance
            double lookahead = Math.max(minLookahead, 
                              Math.min(maxLookahead, minLookahead + lookaheadGain * currentSpeed));
            
            // Get lookahead point
            double lookaheadArcLength = nearestArcLength + lookahead;
            PathSampler.PathPoint lookaheadPoint = pathSampler.pointAtArc(lookaheadArcLength * 1000.0); // Convert back to mm
            
            if (lookaheadPoint == null) {
                return new VelocityCommand(0, 0, 0);
            }
            
            // Convert lookahead point to meters
            double lookaheadXM = lookaheadPoint.x / 1000.0;
            double lookaheadYM = lookaheadPoint.y / 1000.0;
            
            // Translational goal vector (field frame)
            double goalX = lookaheadXM - robotXM;
            double goalY = lookaheadYM - robotYM;
            
            // Desired field velocity
            double vFieldX = translationGain * goalX;
            double vFieldY = translationGain * goalY;
            
            // Rotate to robot frame
            double cosHeading = Math.cos(robotHeading);
            double sinHeading = Math.sin(robotHeading);
            
            double vx = vFieldX * cosHeading + vFieldY * sinHeading;
            double vy = -vFieldX * sinHeading + vFieldY * cosHeading;
            
            // Heading control - use path tangent
            double desiredHeading = lookaheadPoint.heading;
            double headingError = wrapAngle(desiredHeading - robotHeading);
            double omega = headingGain * headingError;
            
            return new VelocityCommand(vx, vy, omega);
        }
        
        private double wrapAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }
        
        public void setLookaheadParameters(double min, double max, double gain) {
            this.minLookahead = min;
            this.maxLookahead = max;
            this.lookaheadGain = gain;
        }
        
        public void setControlGains(double translationGain, double headingGain) {
            this.translationGain = translationGain;
            this.headingGain = headingGain;
        }
    }
    
    // =============================================================================
    // TERMINAL POSE CONTROLLER
    // =============================================================================
    
    private static class TerminalPoseController {
        
        private double xyResponseTime = 0.30; // seconds
        private double headingResponseTime = 0.25; // seconds
        private double maxTerminalVelocity = 0.35; // m/s
        private double maxTerminalOmega = 4.0; // rad/s
        private double positionTolerance = 0.010; // meters (10 mm)
        private double headingTolerance = Math.toRadians(1.0); // 1 degree
        private long dwellTimeMs = 100; // Must hold tolerance for 100ms
        
        private long toleranceStartTime = -1;
        
        public static class VelocityCommand {
            public double vx, vy, omega; // m/s, m/s, rad/s
            
            public VelocityCommand(double vx, double vy, double omega) {
                this.vx = vx; this.vy = vy; this.omega = omega;
            }
        }
        
        public VelocityCommand computeTerminalVelocity(Position goalPose, Position currentPose) {
            // Compute robot-frame error
            double[] robotError = computeRobotFrameError(goalPose, currentPose);
            double ex = robotError[0]; // meters
            double ey = robotError[1]; // meters  
            double eHeading = robotError[2]; // radians
            
            // Control gains from response times
            double kx = 1.0 / xyResponseTime;
            double ky = 1.0 / xyResponseTime;
            double kHeading = 1.0 / headingResponseTime;
            
            // Compute velocities
            double vx = clamp(kx * ex, -maxTerminalVelocity, maxTerminalVelocity);
            double vy = clamp(ky * ey, -maxTerminalVelocity, maxTerminalVelocity);
            double omega = clamp(kHeading * eHeading, -maxTerminalOmega, maxTerminalOmega);
            
            return new VelocityCommand(vx, vy, omega);
        }
        
        public boolean isFinished(Position goalPose, Position currentPose) {
            // Compute errors
            double[] robotError = computeRobotFrameError(goalPose, currentPose);
            double ex = Math.abs(robotError[0]);
            double ey = Math.abs(robotError[1]);
            double eHeading = Math.abs(robotError[2]);
            
            boolean withinTolerance = (ex <= positionTolerance && 
                                      ey <= positionTolerance && 
                                      eHeading <= headingTolerance);
            
            long currentTime = System.currentTimeMillis();
            
            if (withinTolerance) {
                if (toleranceStartTime < 0) {
                    toleranceStartTime = currentTime;
                }
                return (currentTime - toleranceStartTime) >= dwellTimeMs;
            } else {
                toleranceStartTime = -1;
                return false;
            }
        }
        
        public void reset() {
            toleranceStartTime = -1;
        }
        
        private double[] computeRobotFrameError(Position goalPose, Position currentPose) {
            // Field-frame error (convert mm to m)
            double fieldErrorX = (goalPose.getX() - currentPose.getX()) / 1000.0;
            double fieldErrorY = (goalPose.getY() - currentPose.getY()) / 1000.0;
            double headingError = wrapAngle(goalPose.getTheta() - currentPose.getTheta());
            
            // Rotate field error to robot frame
            double cosHeading = Math.cos(currentPose.getTheta());
            double sinHeading = Math.sin(currentPose.getTheta());
            
            double robotErrorX = fieldErrorX * cosHeading + fieldErrorY * sinHeading;
            double robotErrorY = -fieldErrorX * sinHeading + fieldErrorY * cosHeading;
            
            return new double[]{robotErrorX, robotErrorY, headingError};
        }
        
        private double wrapAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }
        
        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
        
        public void setResponseTimes(double xyTime, double headingTime) {
            this.xyResponseTime = xyTime;
            this.headingResponseTime = headingTime;
        }
        
        public void setTerminalLimits(double maxVel, double maxOmega) {
            this.maxTerminalVelocity = maxVel;
            this.maxTerminalOmega = maxOmega;
        }
        
        public void setTolerances(double posTol, double headTol, long dwellMs) {
            this.positionTolerance = posTol;
            this.headingTolerance = headTol;
            this.dwellTimeMs = dwellMs;
        }
    }
    
    // =============================================================================
    // FRICTION ADAPTOR
    // =============================================================================
    
    private static class FrictionAdaptor {
        
        private double windowTime = 0.120;
        private double responseRatioThreshold = 0.8;
        private double targetResponseRatio = 0.95;
        private double speedScalar = 1.0;
        private double minSpeedScalar = 0.6;
        private double maxSpeedScalar = 1.0;
        private double commandedDistanceSum = 0.0;
        private double measuredDistanceSum = 0.0;
        private final double emaAlpha = 0.1;
        private long lastUpdateTime = -1;
        
        public void update(double commandedVx, double commandedVy, 
                          double measuredDeltaX, double measuredDeltaY, 
                          long deltaTimeMs) {
            
            if (lastUpdateTime < 0) {
                lastUpdateTime = System.currentTimeMillis() - deltaTimeMs;
            }
            
            double deltaTimeS = deltaTimeMs / 1000.0;
            
            // Compute commanded displacement
            double commandedDeltaX = commandedVx * deltaTimeS * 1000.0; // Convert to mm
            double commandedDeltaY = commandedVy * deltaTimeS * 1000.0;
            double commandedDistance = Math.hypot(commandedDeltaX, commandedDeltaY);
            
            // Measured displacement magnitude
            double measuredDistance = Math.hypot(measuredDeltaX, measuredDeltaY);
            
            // Update EMA sums
            commandedDistanceSum = (1.0 - emaAlpha) * commandedDistanceSum + emaAlpha * commandedDistance;
            measuredDistanceSum = (1.0 - emaAlpha) * measuredDistanceSum + emaAlpha * measuredDistance;
            
            // Compute response ratio with guard against division by zero
            double responseRatio = 1.0;
            if (commandedDistanceSum > 1e-6) {
                responseRatio = measuredDistanceSum / commandedDistanceSum;
            }
            
            // Update speed scalar based on response ratio
            if (responseRatio < responseRatioThreshold) {
                // Under-responding (slip), decrease scalar
                speedScalar = 0.9 * speedScalar + 0.1 * responseRatioThreshold;
            } else if (responseRatio >= targetResponseRatio) {
                // Good response, gradually increase scalar
                speedScalar = 0.98 * speedScalar + 0.02 * maxSpeedScalar;
            }
            
            // Clamp speed scalar
            speedScalar = Math.max(minSpeedScalar, Math.min(maxSpeedScalar, speedScalar));
            
            lastUpdateTime = System.currentTimeMillis();
        }
        
        public double getSpeedScalar() {
            return speedScalar;
        }
        
        public void scaleVelocity(double[] velocity) {
            velocity[0] *= speedScalar; // vx
            velocity[1] *= speedScalar; // vy
            velocity[2] *= speedScalar; // omega
        }
        
        public void reset() {
            speedScalar = 1.0;
            commandedDistanceSum = 0.0;
            measuredDistanceSum = 0.0;
            lastUpdateTime = -1;
        }
        
        public void setParameters(double windowTime, double threshold, double targetRatio) {
            this.windowTime = windowTime;
            this.responseRatioThreshold = threshold;
            this.targetResponseRatio = targetRatio;
        }
        
        public void setSpeedScalarLimits(double min, double max) {
            this.minSpeedScalar = min;
            this.maxSpeedScalar = max;
        }
    }
    
    // =============================================================================
    // BIAS ADAPTOR
    // =============================================================================
    
    private static class BiasAdaptor {
        
        private double linearIntegrationRate = 0.02; // m/s²
        private double angularIntegrationRate = 0.05; // rad/s²
        private double maxLinearBias = 0.05; // m/s
        private double maxAngularBias = 0.4; // rad/s
        private double biasX = 0.0;
        private double biasY = 0.0;
        private double biasOmega = 0.0;
        private double minStraightOmega = 0.1; // rad/s
        private double minStraightCurvature = 0.5; // 1/m
        private double minPersistentErrorTime = 0.5; // seconds
        private long persistentErrorStartTime = -1;
        
        public void update(double robotErrorX, double robotErrorY, double headingError,
                          double currentOmega, double pathCurvature, double deltaTimeS) {
            
            // Check if conditions are suitable for bias adaptation
            boolean isStraightSegment = Math.abs(currentOmega) < minStraightOmega && 
                                       Math.abs(pathCurvature) < minStraightCurvature;
            
            boolean hasSignificantError = Math.abs(robotErrorX) > 0.005 || // 5mm
                                         Math.abs(robotErrorY) > 0.005 ||  // 5mm
                                         Math.abs(headingError) > Math.toRadians(2.0); // 2 degrees
            
            long currentTime = System.currentTimeMillis();
            
            if (isStraightSegment && hasSignificantError) {
                if (persistentErrorStartTime < 0) {
                    persistentErrorStartTime = currentTime;
                }
                
                double errorDuration = (currentTime - persistentErrorStartTime) / 1000.0;
                
                if (errorDuration >= minPersistentErrorTime) {
                    // Apply bias integration
                    double deltaX = linearIntegrationRate * Math.signum(robotErrorX) * deltaTimeS;
                    double deltaY = linearIntegrationRate * Math.signum(robotErrorY) * deltaTimeS;
                    double deltaOmega = angularIntegrationRate * Math.signum(headingError) * deltaTimeS;
                    
                    biasX = clamp(biasX + deltaX, -maxLinearBias, maxLinearBias);
                    biasY = clamp(biasY + deltaY, -maxLinearBias, maxLinearBias);
                    biasOmega = clamp(biasOmega + deltaOmega, -maxAngularBias, maxAngularBias);
                }
            } else {
                // Reset persistent error timer if conditions not met
                persistentErrorStartTime = -1;
            }
        }
        
        public void applyBias(double[] velocity) {
            velocity[0] += biasX;  // vx
            velocity[1] += biasY;  // vy
            velocity[2] += biasOmega; // omega
        }
        
        public double[] getBias() {
            return new double[]{biasX, biasY, biasOmega};
        }
        
        public void reset() {
            biasX = 0.0;
            biasY = 0.0;
            biasOmega = 0.0;
            persistentErrorStartTime = -1;
        }
        
        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
        
        public void setIntegrationRates(double linearRate, double angularRate) {
            this.linearIntegrationRate = linearRate;
            this.angularIntegrationRate = angularRate;
        }
        
        public void setBiasLimits(double maxLinear, double maxAngular) {
            this.maxLinearBias = maxLinear;
            this.maxAngularBias = maxAngular;
        }
        
        public void setAdaptationConditions(double minOmega, double minCurvature, double minTime) {
            this.minStraightOmega = minOmega;
            this.minStraightCurvature = minCurvature;
            this.minPersistentErrorTime = minTime;
        }
    }
    
    // =============================================================================
    // SPEED CAPS
    // =============================================================================
    
    private static class SpeedCaps {
        
        private double maxCruiseSpeed = 1.2; // m/s
        private double maxBrakingAccel = 2.5; // m/s²
        private double curvatureGain = 0.8;
        
        public void applyCaps(double[] velocity, Position currentPose, Position goalPose, double pathCurvature) {
            
            double currentSpeed = Math.hypot(velocity[0], velocity[1]);
            
            if (currentSpeed < 1e-6) {
                return; // No movement, no caps needed
            }
            
            // 1. Cruise speed cap
            double cruiseCap = maxCruiseSpeed;
            
            // 2. Curvature-based speed cap
            double curvatureCap = maxCruiseSpeed;
            if (Math.abs(pathCurvature) > 1e-6) {
                curvatureCap = maxCruiseSpeed / (1.0 + curvatureGain * Math.abs(pathCurvature));
            }
            
            // 3. Braking distance cap
            double distanceToGoal = currentPose.distanceTo(goalPose) / 1000.0; // Convert mm to m
            double brakingCap = Math.sqrt(2.0 * maxBrakingAccel * distanceToGoal);
            
            // Find the most restrictive cap
            double speedCap = Math.min(cruiseCap, Math.min(curvatureCap, brakingCap));
            
            // Apply cap if current command exceeds it
            if (currentSpeed > speedCap) {
                double scale = speedCap / currentSpeed;
                velocity[0] *= scale;
                velocity[1] *= scale;
                // Don't scale omega - it's independent of translational speed
            }
        }
        
        public double[] getCapValues(Position currentPose, Position goalPose, double pathCurvature) {
            double cruiseCap = maxCruiseSpeed;
            
            double curvatureCap = maxCruiseSpeed;
            if (Math.abs(pathCurvature) > 1e-6) {
                curvatureCap = maxCruiseSpeed / (1.0 + curvatureGain * Math.abs(pathCurvature));
            }
            
            double distanceToGoal = currentPose.distanceTo(goalPose) / 1000.0;
            double brakingCap = Math.sqrt(2.0 * maxBrakingAccel * distanceToGoal);
            
            return new double[]{cruiseCap, curvatureCap, brakingCap};
        }
        
        public void setMaxCruiseSpeed(double speed) {
            this.maxCruiseSpeed = speed;
        }
        
        public void setBrakingAcceleration(double accel) {
            this.maxBrakingAccel = accel;
        }
        
        public void setCurvatureGain(double gain) {
            this.curvatureGain = gain;
        }
    }
    
    // =============================================================================
    // DRIVETRAIN INTEGRATION
    // =============================================================================
    
    private static class DriveTrainIntegration {
        private static final String TAG = "DriveTrainIntegration";
        
        private final DriveTrain driveTrain;
        private boolean isConnected = false;
        private final double maxMotorPower = 1.0;
        private final double motorPowerDeadband = 0.02;
        private final boolean enableSlewRateLimit = true;
        private final double maxPowerChangePerUpdate = 0.1;
        private float[] lastMotorPowers = {0.0f, 0.0f, 0.0f, 0.0f};
        
        public DriveTrainIntegration(DriveTrain driveTrain) {
            this.driveTrain = driveTrain;
            
            if (driveTrain != null) {
                isConnected = true;
                KLog.d(TAG, "DriveTrain integration initialized successfully");
            } else {
                KLog.e(TAG, "DriveTrain is null - integration failed");
            }
        }
        
        public void setMotorPowers(float[] powers) {
            if (!isConnected || powers.length != 4) {
                KLog.e(TAG, "Cannot set motor powers - not connected or invalid array length");
                return;
            }
            
            try {
                // Apply safety limits
                float[] safePowers = applySafetyLimits(powers);
                
                // Apply slew rate limiting if enabled
                if (enableSlewRateLimit) {
                    safePowers = applySlewRateLimit(safePowers);
                }
                
                // Send to DriveTrain
                sendToDriveTrain(safePowers);
                
                // Update last powers for slew rate limiting
                lastMotorPowers = safePowers.clone();
                
            } catch (Exception e) {
                KLog.e(TAG, "Failed to set motor powers", e);
                emergencyStop();
            }
        }
        
        /**
         * MODIFY THIS METHOD to match your DriveTrain interface
         */
        private void sendToDriveTrain(float[] powers) {

            driveTrain.setPower(powers[0], powers[1], powers[2], powers[3]);

            KLog.d(TAG, String.format("Motor Powers: FL=%.3f FR=%.3f BL=%.3f BR=%.3f",
                    powers[0], powers[1], powers[2], powers[3]));
                    
            // TODO: Implement actual motor control based on your DriveTrain interface
        }

        // In NavigationSystem.java, around line 730 in applySafetyLimits method:
        private float[] applySafetyLimits(float[] powers) {
            float[] limitedPowers = new float[4];
            double minMotorPower = 0.18; // Adjust this for your robot!
            for (int i = 0; i < 4; i++) {
                // Clamp to maximum power
                limitedPowers[i] = (float) Math.max(-maxMotorPower, Math.min(maxMotorPower, powers[i]));
                // Apply minimum power threshold
                if (Math.abs(limitedPowers[i]) < motorPowerDeadband) {
                    limitedPowers[i] = 0.0f; // Below deadband = stop
                } else if (Math.abs(limitedPowers[i]) < minMotorPower) {
                    // Apply minimum power while preserving direction
                    limitedPowers[i] = (float) (Math.signum(limitedPowers[i]) * minMotorPower);
                }
            }
            return limitedPowers;
        }

        private float[] applySlewRateLimit(float[] targetPowers) {
            float[] limitedPowers = new float[4];

            for (int i = 0; i < 4; i++) {
                double powerDiff = targetPowers[i] - lastMotorPowers[i];

                if (Math.abs(powerDiff) > maxPowerChangePerUpdate) {
                    // Limit the rate of change
                    double sign = Math.signum(powerDiff);
                    limitedPowers[i] = (float) (lastMotorPowers[i] + sign * maxPowerChangePerUpdate);
                } else {
                    limitedPowers[i] = targetPowers[i];
                }
            }

            return limitedPowers;
        }
        
        public void emergencyStop() {
            try {
                float[] zeroPowers = {0.0f, 0.0f, 0.0f, 0.0f};
                sendToDriveTrain(zeroPowers);
                lastMotorPowers = zeroPowers.clone();
                Log.w(TAG, "Emergency stop executed");
            } catch (Exception e) {
                KLog.e(TAG, "Failed to execute emergency stop", e);
            }
        }
        
        public boolean isConnected() { return isConnected; }
    }
    
    // =============================================================================
    // DATA COLLECTOR
    // =============================================================================
    
    private static class DataCollector {
        private static final String TAG = "DataCollector";
        
        private final List<DataPoint> dataPoints;
        private boolean isCollecting = false;
        private long startTime;
        private String sessionName;
        
        private static class DataPoint {
            public long timestampMs;
            public double timeSeconds;
            public double poseX, poseY, poseHeading;
            public String controlMode;
            public double pathArcLength;
            public double lookaheadDistance;
            public double distanceToGoal;
            public double commandedVx, commandedVy, commandedOmega;
            public double commandedSpeed;
            public double motorFL, motorFR, motorBL, motorBR;
            public double cruiseCap, curvatureCap, brakingCap;
            public double appliedSpeedCap;
            public double frictionScalar;
            public double biasX, biasY, biasOmega;
        }
        
        public DataCollector() {
            dataPoints = new ArrayList<>();
        }
        
        public void startCollection(String sessionName) {
            this.sessionName = sessionName != null ? sessionName : generateSessionName();
            this.dataPoints.clear();
            this.startTime = System.currentTimeMillis();
            this.isCollecting = true;
            
            KLog.d(TAG, "Started data collection: " + this.sessionName);
        }
        
        public void stopCollection() {
            this.isCollecting = false;
            KLog.d(TAG, String.format("Stopped data collection: %s (%d points)", sessionName, dataPoints.size()));
        }
        
        public boolean exportToCSV(String filename) {
            if (dataPoints.isEmpty()) {
                Log.w(TAG, "No data to export");
                return false;
            }
            
            try {
                FileWriter writer = new FileWriter(filename);
                
                // Write header
                writer.append("timestamp_ms,time_s,pose_x,pose_y,pose_heading,")
                      .append("control_mode,path_arc_length,lookahead_distance,distance_to_goal,")
                      .append("commanded_vx,commanded_vy,commanded_omega,commanded_speed,")
                      .append("motor_fl,motor_fr,motor_bl,motor_br,")
                      .append("cruise_cap,curvature_cap,braking_cap,applied_speed_cap,")
                      .append("friction_scalar,bias_x,bias_y,bias_omega\n");
                
                // Write data points - simplified for basic functionality
                for (DataPoint point : dataPoints) {
                    writer.append(String.format(Locale.US,
                        "%d,%.3f,%.2f,%.2f,%.4f," +
                        "%s,%.2f,%.3f,%.3f," +
                        "%.4f,%.4f,%.4f,%.4f," +
                        "%.3f,%.3f,%.3f,%.3f," +
                        "%.3f,%.3f,%.3f,%.3f," +
                        "%.4f,%.4f,%.4f,%.4f\n",
                        
                        point.timestampMs, point.timeSeconds, point.poseX, point.poseY, point.poseHeading,
                        point.controlMode, point.pathArcLength, point.lookaheadDistance, point.distanceToGoal,
                        point.commandedVx, point.commandedVy, point.commandedOmega, point.commandedSpeed,
                        point.motorFL, point.motorFR, point.motorBL, point.motorBR,
                        point.cruiseCap, point.curvatureCap, point.brakingCap, point.appliedSpeedCap,
                        point.frictionScalar, point.biasX, point.biasY, point.biasOmega
                    ));
                }
                
                writer.flush();
                writer.close();
                
                KLog.d(TAG, String.format("Exported %d data points to %s", dataPoints.size(), filename));
                return true;
                
            } catch (IOException e) {
                KLog.e(TAG, "Failed to export data", e);
                return false;
            }
        }
        
        private String generateSessionName() {
            SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
            return "NavData_" + sdf.format(new Date());
        }
        
        public boolean isCollecting() { return isCollecting; }
        public int getDataPointCount() { return dataPoints.size(); }
        public String getSessionName() { return sessionName; }
    }
}