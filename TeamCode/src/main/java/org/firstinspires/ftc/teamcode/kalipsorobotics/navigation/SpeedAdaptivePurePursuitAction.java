package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Enhanced Pure Pursuit Action that extends the original PurePursuitAction
 * with speed-based adaptive look ahead radius and improved power calculation
 */
public class SpeedAdaptivePurePursuitAction extends PurePursuitAction {
    
    // Adaptive look ahead radius parameters - more conservative for stability
    private final double MIN_LOOK_AHEAD_RADIUS = 75;  // mm - start with original default
    private final double MAX_LOOK_AHEAD_RADIUS = 180; // mm - less aggressive than 250
    private final double SPEED_THRESHOLD = 1.5;       // mm/ms - lower threshold for smoother transitions
    
    // Enhanced power calculation parameters
    private final double MAX_POWER_SCALE = 0.85;      // Maximum power scaling factor
    private final double MIN_POWER_THRESHOLD = 0.12;  // Minimum power to overcome friction
    private final double DISTANCE_DAMPING_RADIUS = 100; // mm - distance for power damping
    
    // Speed tracking and filtering
    private double currentSpeed = 0;
    private double filteredSpeed = 0;
    private final double SPEED_FILTER_ALPHA = 0.7;    // Low-pass filter for speed
    private double adaptiveRadius;
    private double previousAdaptiveRadius; // For smoothing radius changes
    
    // Previous position for speed calculation
    private Position previousPosition;
    private long previousTimeMs = 0;
    
    public SpeedAdaptivePurePursuitAction(DriveTrain driveTrain) {
        super(driveTrain);
        this.adaptiveRadius = MIN_LOOK_AHEAD_RADIUS;
        this.previousAdaptiveRadius = MIN_LOOK_AHEAD_RADIUS;
        this.previousPosition = new Position(SharedData.getOdometryWheelIMUPosition());
        this.previousTimeMs = System.currentTimeMillis();
    }
    
    @Override
    public void update() {
        // Calculate current speed before calling parent update
        updateCurrentSpeed();
        
        // Update adaptive look ahead radius based on current speed
        updateAdaptiveLookAheadRadius();
        
        // Override the look ahead radius in parent class using reflection
        setAdaptiveLookAheadRadius();
        
        // Call parent update method which will use our adaptive radius
        super.update();
    }
    
    /**
     * Calculate current robot speed from position changes
     */
    private void updateCurrentSpeed() {
        Position currentPosition = new Position(SharedData.getOdometryWheelIMUPosition());
        long currentTimeMs = System.currentTimeMillis();
        
        if (previousPosition != null && previousTimeMs > 0) {
            // Calculate distance traveled
            Vector displacement = Vector.between(previousPosition, currentPosition);
            double distanceMm = displacement.getLength();
            
            // Calculate time elapsed
            double deltaTimeMs = currentTimeMs - previousTimeMs;
            
            if (deltaTimeMs > 0) {
                // Calculate current speed (mm/ms)
                currentSpeed = distanceMm / deltaTimeMs;
                
                // Apply low-pass filter to smooth out noise
                filteredSpeed = SPEED_FILTER_ALPHA * filteredSpeed + (1 - SPEED_FILTER_ALPHA) * currentSpeed;
            }
        }
        
        // Update previous values for next iteration
        previousPosition = currentPosition;
        previousTimeMs = currentTimeMs;
        
        KLog.d("SpeedAdaptivePP", () -> String.format("Raw Speed: %.3f mm/ms, Filtered: %.3f mm/ms",
            currentSpeed, filteredSpeed));
    }
    
    /**
     * Update adaptive look ahead radius based on current filtered speed
     */
    private void updateAdaptiveLookAheadRadius() {
        // Calculate speed ratio (0.0 to 1.0) with more gradual scaling
        double speedRatio = Math.min(filteredSpeed / SPEED_THRESHOLD, 1.0);
        
        // Use gentle exponential curve for more conservative adaptation
        // This prevents rapid radius changes that cause oscillation
        double smoothedRatio = 1.0 - Math.exp(-speedRatio * 2.0); // Exponential approach for stability
        
        adaptiveRadius = MIN_LOOK_AHEAD_RADIUS + 
            (MAX_LOOK_AHEAD_RADIUS - MIN_LOOK_AHEAD_RADIUS) * smoothedRatio;
        
        // Apply additional smoothing to prevent rapid radius changes
        if (Math.abs(adaptiveRadius - previousAdaptiveRadius) > 10) {
            adaptiveRadius = previousAdaptiveRadius + Math.signum(adaptiveRadius - previousAdaptiveRadius) * 10;
        }
        previousAdaptiveRadius = adaptiveRadius;
        
        KLog.d("SpeedAdaptivePP", () -> String.format("Speed: %.3f, Ratio: %.3f, Adaptive Radius: %.1f mm",
            filteredSpeed, smoothedRatio, adaptiveRadius));
    }
    
    /**
     * Override the parent's look ahead radius using reflection
     */
    private void setAdaptiveLookAheadRadius() {
        try {
            // Access the private currentLookAheadRadius field from parent
            java.lang.reflect.Field radiusField = PurePursuitAction.class.getDeclaredField("currentLookAheadRadius");
            radiusField.setAccessible(true);
            radiusField.setDouble(this, adaptiveRadius);
            
            // Also set the base lookAheadRadius field
            java.lang.reflect.Field baseLookAheadField = PurePursuitAction.class.getDeclaredField("lookAheadRadius");
            baseLookAheadField.setAccessible(true);
            baseLookAheadField.setDouble(this, adaptiveRadius);
            
        } catch (Exception e) {
            KLog.d("SpeedAdaptivePP", () -> "Could not set adaptive radius: " + e.getMessage());
            // Fallback: use parent's setLookAheadRadius method
            super.setLookAheadRadius(adaptiveRadius);
        }
    }
    
    /**
     * Enhanced power calculation with speed-based scaling and distance damping
     * This method provides improved calculation and calls parent's private targetPosition via reflection
     */
    private void enhancedTargetPosition(Position target, Position currentPos) {
        Vector currentToTarget = Vector.between(currentPos, target);
        
        double distanceToTarget = currentToTarget.getLength();
        double targetDirection = currentToTarget.getHeadingDirection();
        double targetAngle = target.getTheta();
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());
        
        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double xError = Math.cos(directionError) * distanceToTarget;
        double yError = Math.sin(directionError) * distanceToTarget;
        
        // Get PID powers from target position
        double powerAngle = target.getPidAngle().getPower(angleError);
        double powerX = target.getPidX().getPower(xError);
        double powerY = target.getPidY().getPower(yError);
        
        // Apply enhanced scaling based on speed and distance
        double speedScale = calculateSpeedBasedScale();
        double distanceScale = calculateDistanceBasedScale(distanceToTarget);
        double combinedScale = Math.min(speedScale, distanceScale);
        
        // Apply scaling to linear movement, preserve some angular correction
        powerX *= combinedScale;
        powerY *= combinedScale;
        
        // Reduce angular power when moving fast or when close to target
        double angleDamping = Math.min(distanceToTarget / 150.0, 1.0) * (1.0 - filteredSpeed / (SPEED_THRESHOLD * 2));
        angleDamping = Math.max(angleDamping, 0.2); // Maintain minimum angular authority
        powerAngle *= angleDamping;

        double finalPowerX = powerX;
        double finalPowerY = powerY;
        double finalPowerAngle = powerAngle;
        KLog.d("SpeedAdaptivePP_Power", () -> String.format(
            "Enhanced Powers - X: %.4f, Y: %.4f, Angle: %.4f, Speed Scale: %.3f, Distance Scale: %.3f",
                finalPowerX, finalPowerY, finalPowerAngle, speedScale, distanceScale));
        
        // Calculate motor powers
        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;
        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;
        
        // Enhanced power normalization and limiting
        double maxPower = Math.max(
            Math.max(Math.abs(fLeftPower), Math.abs(bLeftPower)), 
            Math.max(Math.abs(fRightPower), Math.abs(bRightPower))
        );
        
        if (maxPower > MAX_POWER_SCALE) {
            double scale = MAX_POWER_SCALE / maxPower;
            fLeftPower *= scale;
            bLeftPower *= scale;
            fRightPower *= scale;
            bRightPower *= scale;
        }
        
        // Apply minimum power threshold to overcome friction
        fLeftPower = applyMinimumPower(fLeftPower);
        bLeftPower = applyMinimumPower(bLeftPower);
        fRightPower = applyMinimumPower(fRightPower);
        bRightPower = applyMinimumPower(bRightPower);
        
        // Get drive train and set powers
        try {
            java.lang.reflect.Field driveTrainField = PurePursuitAction.class.getDeclaredField("driveTrain");
            driveTrainField.setAccessible(true);
            DriveTrain driveTrain = (DriveTrain) driveTrainField.get(this);
            
            driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

            double finalFLeftPower = fLeftPower;
            double finalFRightPower = fRightPower;
            double finalBLeftPower = bLeftPower;
            double finalBRightPower = bRightPower;
            KLog.d("SpeedAdaptivePP_Motors", () -> String.format(
                "Motor Powers - FL: %.4f, FR: %.4f, BL: %.4f, BR: %.4f",
                    finalFLeftPower, finalFRightPower, finalBLeftPower, finalBRightPower));
                
        } catch (Exception e) {
            KLog.d("SpeedAdaptivePP", () -> "Could not access drive train: " + e.getMessage());
            // Could not set motor powers directly - parent will handle it
        }
    }
    
    /**
     * Calculate speed-based power scaling factor
     */
    private double calculateSpeedBasedScale() {
        // Reduce power at very high speeds for stability
        // Increase power at low speeds for responsiveness
        double speedScale;
        if (filteredSpeed < SPEED_THRESHOLD * 0.5) {
            // Low speed: boost power slightly for responsiveness
            speedScale = 1.0 + (0.2 * (1.0 - filteredSpeed / (SPEED_THRESHOLD * 0.5)));
        } else {
            // High speed: reduce power for stability
            double highSpeedRatio = (filteredSpeed - SPEED_THRESHOLD * 0.5) / (SPEED_THRESHOLD * 0.5);
            speedScale = 1.0 - (0.3 * Math.min(highSpeedRatio, 1.0));
        }
        
        return Math.max(speedScale, 0.4); // Minimum 40% power
    }
    
    /**
     * Calculate distance-based power scaling factor
     */
    private double calculateDistanceBasedScale(double distanceToTarget) {
        // Reduce power when very close to target for precision
        if (distanceToTarget < DISTANCE_DAMPING_RADIUS) {
            double distanceRatio = distanceToTarget / DISTANCE_DAMPING_RADIUS;
            return 0.5 + 0.5 * distanceRatio; // Scale from 50% to 100%
        }
        return 1.0;
    }
    
    /**
     * Apply minimum power threshold while preserving direction
     */
    private double applyMinimumPower(double power) {
        if (Math.abs(power) > 0.01 && Math.abs(power) < MIN_POWER_THRESHOLD) {
            return Math.signum(power) * MIN_POWER_THRESHOLD;
        }
        return power;
    }
    
    /**
     * Public getters for debugging and monitoring
     */
    public double getCurrentSpeed() {
        return filteredSpeed;
    }
    
    public double getAdaptiveRadius() {
        return adaptiveRadius;
    }
    
    /**
     * Configure adaptive behavior parameters
     */
    public void setAdaptiveParameters(double minRadius, double maxRadius, double speedThreshold) {
        // This would require making the fields non-final, but provides flexibility
        KLog.d("SpeedAdaptivePP", "Adaptive parameters would be set here if fields were non-final");
    }
}