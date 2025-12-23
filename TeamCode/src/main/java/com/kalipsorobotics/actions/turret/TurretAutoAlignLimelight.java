package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.test.turret.TurretRunMode;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TurretAutoAlignLimelight extends Action {
    OpModeUtilities opModeUtilities;
    Turret turret;
    KMotor turretMotor;
    private AprilTagDetectionAction aprilTagDetectionAction;
    private TurretRunMode turretRunMode;
    private double targetTicks;
    private double targetPower;
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


    public TurretAutoAlignLimelight(OpModeUtilities opModeUtilities, Turret turret, AprilTagDetectionAction aprilTagDetectionAction, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.aprilTagDetectionAction = aprilTagDetectionAction;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.turretRunMode = TurretRunMode.STOP; // inital mode
        this.targetTicks = 0;
        this.targetPower = 0;


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

        switch (turretRunMode) {
            case STOP:
                turret.stop();
                isWithinRange = true;
                return;

            case RUN_WITH_POWER:
                turretMotor.setPower(targetPower);
                isWithinRange = true;
                return;

            case RUN_USING_ODOMETRY_AND_LL:
                updateAlignToTarget();
                return;
        }
    }

    private void updateAlignToTarget() {
        aprilTagDetectionAction.updateCheckDone();
        updateAngularVelocity();
        aprilTagFound = !SharedData.getLimelightPosition().isEmpty();
        double currentAngleRad = turret.getCurrentAngleRad();

        if (aprilTagFound) {
            double targetAngleLimelight = SharedData.getLimelightPosition().getAngleToGoalRad();
            hasSearched = false;
            turretMotor.getPIDFController().setKp(TurretConfig.kP);
            totalAngleWrap = MathFunctions.angleWrapRad(currentAngleRad - targetAngleLimelight);
            runningMode = TurretRunningMode.GO_TO_NEAREST_LIMIT;
            targetTicks = totalAngleWrap * Turret.TICKS_PER_RADIAN;
            KLog.d("TurretAutoAlignLimelight_Camera_Calc", "targetTicks " + targetTicks);
            KLog.d("TurretAutoAlignLimelight", "Limelight angle " + targetAngleLimelight);
        } else if (useOdometrySearch) {
            targetTicks = TurretAutoAlign.calculateTargetTicks(targetPoint);
            KLog.d("TurretAutoAlignLimelight_Odometry_Calc", "targetTicks " + targetTicks);
        } else {
            KLog.d("TurretAutoAlignLimelight", "No april tag and odometry search disabled");
            return;
        }

        moveToTargetTicks();
    }

    private void moveToTargetTicks() {
        int currentTicks = turretMotor.getCurrentPosition();
        int error = (int) targetTicks - currentTicks;

        if (Math.abs(error) < Math.abs(toleranceTicks)) {
            isWithinRange = true;
            turretMotor.stop();
            KLog.d("TurretAlign_WithinRange", String.format("Within RANGE - current: %d ticks, target: %.0f ticks", currentTicks, targetTicks));
        } else {
            isWithinRange = false;

            double pidOutput = turretMotor.getPIDFController().calculate(error);

            double feedforward = TurretConfig.kF * currentAngularVelocity;

            double totalPower = Math.max(-1.0, Math.min(1.0, pidOutput + feedforward));

            KLog.d("TurretAlign_Power", String.format("Target: %.0f, Current: %d, Error: %d, PID: %.3f, FF: %.3f, Total: %.3f",
                targetTicks, currentTicks, error, pidOutput, feedforward, totalPower));

            turretMotor.setPower(totalPower);
        }
    }

    public void runWithPower(double power) {
        this.turretRunMode = TurretRunMode.RUN_WITH_POWER;
        this.targetPower = power;
    }
    public void runWithCurrentPos() {
        this.turretRunMode = TurretRunMode.RUN_USING_ODOMETRY_AND_LL;
    }

    public void stop() {
        this.turretRunMode = TurretRunMode.STOP;
    }

    public void stopAndSetDone() {
        turretMotor.setPower(0);
        isDone = true;
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
        Position currentPosition = SharedData.getOdometryWheelIMUPosition();
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
            double deltaTime = velocityTimer.milliseconds() ;
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
