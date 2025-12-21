package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TurretAutoAlignLimelight extends Action {
    OpModeUtilities opModeUtilities;

    Turret turret;
    KMotor turretMotor;
    private GoalDetectionAction goalDetectionAction;

    private double targetTicks;
    private final double DEFAULT_TOLERANCE_TICKS = (Turret.TICKS_PER_DEGREE)*1;
    private double toleranceTicks = DEFAULT_TOLERANCE_TICKS;
    private boolean isWithinRange = false;
    private boolean aprilTagFound = false;
    private double searchAngleDeg = 180;
    private boolean hasSearched = false;
    double totalAngleWrap;
    double lastLimit;
    TurretRunningMode runningMode;

    Point targetPoint;
    private boolean useOdometrySearch;
    private double previousTotalAngle;
    private double currentAngularVelocity;
    private ElapsedTime velocityTimer;
    private boolean isFirstVelocityUpdate;


    public TurretAutoAlignLimelight(OpModeUtilities opModeUtilities, Turret turret, GoalDetectionAction goalDetectionAction, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.goalDetectionAction = goalDetectionAction;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.targetTicks = 0;


        targetPoint = new Point(TurretConfig.X_INIT_SETUP_MM, TurretConfig.Y_INIT_SETUP_MM * allianceColor.getPolarity());

        // Initialize velocity tracking
        this.velocityTimer = new ElapsedTime();
        this.previousTotalAngle = 0;
        this.currentAngularVelocity = 0;
        this.isFirstVelocityUpdate = true;
    }

    public boolean isWithinRange() {
        return isWithinRange;
    }

    public void stop() {
        turretMotor.stop();
    }
    public void initBlocking() {
        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (timer.milliseconds() < 3000) {
            count++;
            if (timer.milliseconds() > 1000) {
                this.updateCheckDone();
            }
            opModeUtilities.getTelemetry().addData("count ", count);
            opModeUtilities.getTelemetry().addLine("TURRET IS ALIGNING WAIT");
            opModeUtilities.getTelemetry().update();
        }
        opModeUtilities.getTelemetry().addData("isWithinRange ", isWithinRange);
        opModeUtilities.getTelemetry().addLine("TURRET ALIGNED READY");
        opModeUtilities.getTelemetry().update();
    }

    public Turret getTurret() {
        return turret;
    }

    @Override
    protected void update() {
        if (!opModeUtilities.getOpMode().opModeIsActive() && !opModeUtilities.getOpMode().opModeInInit()) {
            KLog.d("Turret_Singleton", "OpModeNotActive Return");
            turretMotor.stop();
            return;
        }

        if (!hasStarted) {
            hasStarted = true;
            velocityTimer.reset();
            runningMode = TurretRunningMode.GO_TO_NEAREST_LIMIT;
        }

        updateAngularVelocity();
        goalDetectionAction.updateCheckDone();
        aprilTagFound = !SharedData.getLimelightPosition().isEmpty();
        double targetAngleLimelight = 0;
        double currentAngleRad = turret.getCurrentAngleRad();

        if (SharedData.getLimelightPosition().isEmpty() && useOdometrySearch) {
            /*//searching
            turretMotor.getPIDFController().setKp(TurretConfig.kP / 10);

            if (runningMode == TurretRunningMode.GO_TO_NEAREST_LIMIT) {
                //runing mode - go to neaeres limit
                if (currentAngleRad < 0) {
                    totalAngleWrap = Math.toRadians(-180);
                }
                if (currentAngleRad > 0) {
                    totalAngleWrap = Math.toRadians(180);
                }

                lastLimit = totalAngleWrap;

                if (Math.abs(turretMotor.getCurrentPosition() - targetTicks) < Math.abs(toleranceTicks)) {
                   runningMode = TurretRunningMode.GO_TO_CENTER;
                }
            }

            if (runningMode == TurretRunningMode.GO_TO_CENTER) {
                totalAngleWrap = 0;
                if (Math.abs(turretMotor.getCurrentPosition() - targetTicks) < Math.abs(toleranceTicks)) {
                    runningMode = TurretRunningMode.GO_TO_OPPOSITE_LIMIT;
                }
            }
            if (runningMode == TurretRunningMode.GO_TO_OPPOSITE_LIMIT){
                totalAngleWrap = lastLimit * -1;
                if (Math.abs(turretMotor.getCurrentPosition() - targetTicks) < Math.abs(toleranceTicks)) {
                    runningMode = TurretRunningMode.GO_TO_CENTER;
                    lastLimit = totalAngleWrap;
                }
            }*/
            targetTicks = TurretAutoAlign.calculateTargetTicks(targetPoint);
            KLog.d("TurretAutoAlignLimelight_Odometry_Calc", "targetTicks " + targetTicks);

        } else {
            //April tag found - locking angle
            targetAngleLimelight = SharedData.getLimelightPosition().getAngleToGoalRad();
            hasSearched = false;
            turretMotor.getPIDFController().setKp(TurretConfig.kP);
            totalAngleWrap = MathFunctions.angleWrapRad(currentAngleRad - targetAngleLimelight);
            runningMode = TurretRunningMode.GO_TO_NEAREST_LIMIT;
            targetTicks = totalAngleWrap * Turret.TICKS_PER_RADIAN;
            KLog.d("TurretAutoAlignLimelight_Camera_Calc", "targetTicks " + targetTicks);
        }
        KLog.d("TurretAutoAlignLimelight", "Limelight angle" + targetAngleLimelight);

//        double turretRotation = (totalAngleWrap) / (2 * Math.PI);
//        double motorRotation = turretRotation * Turret.BIG_TO_SMALL_PULLEY;
//        targetTicks = Turret.TICKS_PER_ROTATION * motorRotation;


        if (Math.abs(turretMotor.getCurrentPosition() - targetTicks) < Math.abs(toleranceTicks)) {
            isWithinRange = true;
            turretMotor.stop();
            KLog.d("AngVel_withinRange", String.format("Within RANGE - motor position %.2f, target ticks %.2f", (double) turretMotor.getCurrentPosition(), targetTicks));

        } else {
            isWithinRange = false;
//             turretMotor.goToTargetTicks((int) targetTicks);

            int currentTicks = turretMotor.getCurrentPosition();
            int error = (int) targetTicks - currentTicks;

            // PID for position control
            double pidOutput = turretMotor.getPIDFController().calculate(error);

            // Velocity feedforward - helps turret track smoothly as robot/target moves
            double feedforward = TurretConfig.kF * currentAngularVelocity;

            double totalPower = Math.max(-1.0, Math.min(1.0, pidOutput + feedforward));

            KLog.d("AngVel_Power", String.format("Error: %d ticks, PID: %.3f, FF: %.3f (AngVel: %.6f rad/ms), Total: %.3f",
                error, pidOutput, feedforward, currentAngularVelocity, totalPower));

            turretMotor.setPower(totalPower);

        }
    }


    public double getTargetTicks() {
        return targetTicks;
    }

    public void setToleranceDeg(double newToleranceDeg) {
        toleranceTicks = newToleranceDeg * Turret.TICKS_PER_DEGREE;
    }

    public boolean isAprilTagFound() {
        return aprilTagFound;
    }

    public void setUseOdometrySearch(boolean useOdometrySearch) {
        this.useOdometrySearch = useOdometrySearch;
    }

    private void updateAngularVelocity() {
        Position currentPosition = SharedData.getOdometryPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();
        double currentRobotHeading = currentPosition.getTheta();

        // Log robot position and target
        KLog.d("AngVel", String.format("Robot: (%.1f, %.1f) mm, Heading: %.3f rad (%.1f deg)",
            currentX, currentY, currentRobotHeading, Math.toDegrees(currentRobotHeading)));
        KLog.d("AngVel", String.format("Target: (%.1f, %.1f) mm",
            targetPoint.getX(), targetPoint.getY()));

        double yToGoal = targetPoint.getY() - currentY;
        double xToGoal = targetPoint.getX() - currentX;
        double distanceToGoal = Math.sqrt(xToGoal * xToGoal + yToGoal * yToGoal);
        double angleToGoal = Math.atan2(yToGoal, xToGoal);

        // Log vector to goal
        KLog.d("AngVel", String.format("Delta: (%.1f, %.1f) mm, Distance: %.1f mm, Angle: %.3f rad (%.1f deg)",
            xToGoal, yToGoal, distanceToGoal, angleToGoal, Math.toDegrees(angleToGoal)));

        // total angle
        double totalAngleToGoal = MathFunctions.angleWrapRad(angleToGoal - currentRobotHeading);
        KLog.d("AngVel", String.format("Angle Error: %.3f rad (%.1f deg) = angleToGoal(%.3f) - robotHeading(%.3f)",
            totalAngleToGoal, Math.toDegrees(totalAngleToGoal), angleToGoal, currentRobotHeading));

        if (isFirstVelocityUpdate) {
            // first time
            previousTotalAngle = totalAngleToGoal;
            currentAngularVelocity = 0;
            velocityTimer.reset();
            isFirstVelocityUpdate = false;
            KLog.d("AngVel", "First update - initializing velocity tracking");
        } else {
            // update velocity
            double deltaTime = velocityTimer.milliseconds() ; // Convert to seconds (bruh, for what?)
            if (deltaTime > 0) {
                double deltaAngle = totalAngleToGoal - previousTotalAngle;
                currentAngularVelocity = deltaAngle / deltaTime;

                // Log velocity calculation components
                KLog.d("AngVel", String.format("ΔAngle: %.4f rad, ΔTime: %.1f ms, Velocity: %.3f rad/ms (%.3f rad/s)",
                    deltaAngle, deltaTime, currentAngularVelocity, currentAngularVelocity * 1000.0));

                previousTotalAngle = totalAngleToGoal;
                velocityTimer.reset();
            }
        }

        // Summary log
        KLog.d("AngVel", String.format("Angular Velocity: %.3f rad/ms, Angle Error: %.3f rad (%.1f deg)",
            currentAngularVelocity, totalAngleToGoal, Math.toDegrees(totalAngleToGoal)));
    }
}
