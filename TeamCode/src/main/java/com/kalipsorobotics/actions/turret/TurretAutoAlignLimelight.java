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
    private double odometryTransitionThresholdCount = 20;
    private double odometryTransitionCount = 0;
    double totalAngleWrap;
    double lastLimit;
    Point targetPoint;
    private boolean useOdometryAlign;
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

        KLog.d("TurretStateMachine", "CONSTRUCTOR called - initial mode: " + turretRunMode);
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
        KLog.d("TurretStateMachine", "update() called - turretRunMode: " + turretRunMode);

        if (!opModeUtilities.getOpMode().opModeIsActive() && !opModeUtilities.getOpMode().opModeInInit()) {
            KLog.d("TurretStateMachine", "OpMode NOT active and NOT in init - returning early");
            turretMotor.stop();
            return;
        }

        if (!hasStarted) {
            hasStarted = true;
            velocityTimer.reset();
            KLog.d("TurretStateMachine", "First update - hasStarted set to true");
        }

        KLog.d("TurretStateMachine", "Switching on turretRunMode: " + turretRunMode);

        switch (turretRunMode) {
            case STOP:
                KLog.d("TurretStateMachine", "STOP mode - calling turret.stop()");
                turret.stop();
                isWithinRange = true;
                break;
            case RUN_WITH_POWER:
                turretMotor.setPower(targetPower);
                KLog.d("TurretStateMachine", "RUN_WITH_MANUAL_CONTROL mode - manualTargetTicks: " + targetPower + ", power: " + turretMotor.getPower());
                isWithinRange = true;
                break;
            case RUN_USING_LIMELIGHT:
                KLog.d("TurretStateMachine", "RUN_USING_LimeLight mode - useOdometrySearch=false");
                useOdometryAlign = false;
                updateAlignToTarget();
                break;
            case RUN_USING_ODOMETRY_AND_LIMELIGHT:
                KLog.d("TurretStateMachine", "RUN_USING_ODOMETRY_AND_LimeLight mode - useOdometrySearch=true");
                useOdometryAlign = true;
                updateAlignToTarget();
                break;
        }
        KLog.d("TurretStateMachine", "CURRENT TURRET POWER IS: " + turretMotor.getPower());
        KLog.d("TurretStateMachine", "update() complete - isWithinRange: " + isWithinRange);
    }

    private void updateAlignToTarget() {
        KLog.d("TurretStateMachine", "updateAlignToTarget() - useOdometrySearch: " + useOdometryAlign);

        aprilTagDetectionAction.updateCheckDone();
        updateAngularVelocity();
        aprilTagFound = !SharedData.getLimelightRawPosition().isEmpty();
        double currentAngleRad = turret.getCurrentAngleRad();

        KLog.d("TurretStateMachine", "aprilTagFound: " + aprilTagFound + ", currentAngleRad: " + currentAngleRad);

        // ONLY FOR DASHBOARD TUNING TO UPDATE WITHOUT RECONSTUCTING
        // -------------------------------------------
        turretMotor.getPIDFController().setKp(TurretConfig.kP);
        turretMotor.getPIDFController().setKf(TurretConfig.kF);
        turretMotor.getPIDFController().setKs(TurretConfig.kS);
        // -------------------------------------------

        if (aprilTagFound) {
            hasSearched = false;
            odometryTransitionCount = 0;
            targetTicks = TurretAutoAlign.calculateTargetTicks(targetPoint, SharedData.getLimelightGlobalPosition());
            KLog.d("TurretStateMachine", "Using LIMELIGHT targetTicks: " + targetTicks);
        } else if (useOdometryAlign) {
            odometryTransitionCount++;
            KLog.d("TurretStateMachine", "Limelight not seen, odometryTransitionCount: " + odometryTransitionCount + "/" + odometryTransitionThresholdCount);
            if (odometryTransitionCount > odometryTransitionThresholdCount) {
                targetTicks = TurretAutoAlign.calculateTargetTicks(targetPoint, SharedData.getOdometryWheelIMUPosition());
                KLog.d("TurretStateMachine", "Using ODOMETRY - targetTicks: " + targetTicks);
            } else {
                KLog.d("TurretStateMachine", "Waiting for transition threshold, holding position");
                return;
            }
        } else {
            KLog.d("TurretStateMachine", "No april tag and odometry disabled - holding position");
            turretMotor.setPower(0);
            isWithinRange = true;
            return;
        }

        KLog.d("TurretStateMachine", "Calling moveToTargetTicks() - targetTicks: " + targetTicks);
        moveToTargetTicks();
    }

    private void moveToTargetTicks() {
        int currentTicks = turretMotor.getCurrentPosition();
        int error = (int) targetTicks - currentTicks;

        KLog.d("TurretStateMachine", "moveToTargetTicks() - currentTicks: " + currentTicks + ", targetTicks: " + targetTicks + ", error: " + error + ", tolerance: " + toleranceTicks);

        if (Math.abs(error) < Math.abs(toleranceTicks)) {
            isWithinRange = true;
            turretMotor.stop();
            KLog.d("TurretStateMachine", "WITHIN RANGE - stopping motor");
        } else {
            isWithinRange = false;

            double pidOutput = turretMotor.getPIDFController().calculate(error);

            double feedforward = TurretConfig.kF * currentAngularVelocity;

            double totalPower = Math.max(-1.0, Math.min(1.0, pidOutput + feedforward));

            KLog.d("TurretStateMachine", String.format("MOVING - PID: %.3f, FF: %.3f, totalPower: %.3f", pidOutput, feedforward, totalPower));

            turretMotor.setPower(totalPower);
        }
    }

    public void runWithPower(double power) {
        KLog.d("TurretStateMachine", "runWithTicksIncrement() called - power: " + power + ", previous mode: " + turretRunMode);
        this.turretRunMode = TurretRunMode.RUN_WITH_POWER;
        this.targetPower = power;
        KLog.d("TurretStateMachine", "runWithTicksIncrement() - mode now: " + turretRunMode + ", manualTargetTicks: " + targetPower);
    }

    public void runWithOdometryAndLimelight() {
        KLog.d("TurretStateMachine", "runWithOdometryAndLimelight() called - previous mode: " + turretRunMode);
        this.turretRunMode = TurretRunMode.RUN_USING_ODOMETRY_AND_LIMELIGHT;
        KLog.d("TurretStateMachine", "runWithOdometryAndLimelight() - mode now: " + turretRunMode);
    }

    public void runWithLimelight() {
        KLog.d("TurretStateMachine", "runWithLimelight() called - previous mode: " + turretRunMode);
        this.turretRunMode = TurretRunMode.RUN_USING_LIMELIGHT;
        KLog.d("TurretStateMachine", "runWithLimelight() - mode now: " + turretRunMode);
    }

    public void stop() {
        KLog.d("TurretStateMachine", "stop() called - previous mode: " + turretRunMode);
        this.turretRunMode = TurretRunMode.STOP;
        KLog.d("TurretStateMachine", "stop() - mode now: " + turretRunMode);
    }

    public double getTargetTicks() {
        return targetTicks;
    }

    public void setToleranceDeg(double newToleranceDeg) {
        toleranceTicks = newToleranceDeg * Turret.TICKS_PER_DEGREE;
    }

    private void updateAngularVelocity() {
        Position currentPosition = SharedData.getOdometryWheelIMUPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();
        double currentRobotHeading = currentPosition.getTheta();

        double yToGoal = targetPoint.getY() - currentY;
        double xToGoal = targetPoint.getX() - currentX;
        double distanceToGoal = Math.sqrt(xToGoal * xToGoal + yToGoal * yToGoal);
        double angleToGoal = Math.atan2(yToGoal, xToGoal);

        double totalAngleToGoal = MathFunctions.angleWrapRad(angleToGoal - currentRobotHeading);

        if (isFirstVelocityUpdate) {
            previousTotalAngle = totalAngleToGoal;
            currentAngularVelocity = 0;
            velocityTimer.reset();
            isFirstVelocityUpdate = false;
        } else {
            double deltaTime = velocityTimer.milliseconds();
            if (deltaTime > 0) {
                double deltaAngle = totalAngleToGoal - previousTotalAngle;
                currentAngularVelocity = deltaAngle / deltaTime;
                previousTotalAngle = totalAngleToGoal;
                velocityTimer.reset();
            }
        }

        KLog.d("TurretStateMachine", String.format("AngVel: %.4f rad/ms, AngleToGoal: %.1f deg, Distance: %.0f mm",
            currentAngularVelocity, Math.toDegrees(totalAngleToGoal), distanceToGoal));
    }

    public boolean getAlignWithOdometry() {
        return useOdometryAlign;
    }
}
