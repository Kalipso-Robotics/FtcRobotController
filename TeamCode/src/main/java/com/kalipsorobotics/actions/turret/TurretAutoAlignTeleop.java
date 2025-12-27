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


public class TurretAutoAlignTeleop extends Action {
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
    Point targetPoint;
    private boolean useOdometryAlign;
    private double previousTotalAngle;
    private double currentAngularVelocity;
    private ElapsedTime velocityTimer;
    private boolean isFirstVelocityUpdate;


    public TurretAutoAlignTeleop(OpModeUtilities opModeUtilities, Turret turret, AprilTagDetectionAction aprilTagDetectionAction, AllianceColor allianceColor) {
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
            turretMotor.stop();
            return;
        }

        if (!hasStarted) {
            hasStarted = true;
            velocityTimer.reset();
        }

        switch (turretRunMode) {
            case STOP:
                turret.stop();
                isWithinRange = true;
                break;
            case RUN_WITH_POWER:
                turretMotor.setPower(targetPower);
                isWithinRange = true;
                break;
            case RUN_USING_LIMELIGHT:
                useOdometryAlign = false;
                isWithinRange = false;
                updateAlignToTarget();
                break;
            case RUN_USING_ODOMETRY_AND_LIMELIGHT:
                useOdometryAlign = true;
                isWithinRange = false;
                updateAlignToTarget();
                break;
        }
    }

    private void updateAlignToTarget() {
        aprilTagDetectionAction.updateCheckDone();
        updateAngularVelocity();
        aprilTagFound = !SharedData.getLimelightRawPosition().isEmpty();

        // Dashboard tuning support
        turretMotor.getPIDFController().setKp(TurretConfig.kP);
        turretMotor.getPIDFController().setKf(TurretConfig.kF);
        turretMotor.getPIDFController().setKs(TurretConfig.kS);

        // ==================== RAW: Turret Encoder ====================
        double currentAngleRad = turret.getCurrentAngleRad();

        if (aprilTagFound) {
            double targetAngleLimelight = SharedData.getLimelightRawPosition().getGoalAngleToCamRad();
            double reversedTurretAngleRad = -currentAngleRad;
//            hasSearched = false;
            totalAngleWrap = MathFunctions.angleWrapRad(reversedTurretAngleRad + targetAngleLimelight);
            // ==================== RAW: Limelight Angle ====================
            double limelightAngleRad = SharedData.getLimelightRawPosition().getGoalAngleToCamRad();

            // ==================== CALC: Target Position (Limelight) ====================
            totalAngleWrap = MathFunctions.angleWrapRad(currentAngleRad + limelightAngleRad);
            targetTicks = totalAngleWrap * Turret.TICKS_PER_RADIAN;

            KLog.d("Turret_LIMELIGHT", String.format("RAW: TurretPos=%.2f° LLAngle=%.2f° | CALC: Target=%.2f° (%d ticks)",
                    Math.toDegrees(currentAngleRad), Math.toDegrees(limelightAngleRad),
                    Math.toDegrees(totalAngleWrap), (int) targetTicks));

        } else if (useOdometryAlign) {
            // ==================== RAW: Odometry Data ====================
            Position odomPos = SharedData.getOdometryWheelIMUPosition();
            double robotAngleRad = odomPos.getTheta();
            double turretAngleRad = turret.getCurrentAngleRad();

            // ==================== CALC: Bias Angle ====================
            double biasAngle = Math.atan2(targetPoint.getY(), targetPoint.getX());

            // ==================== CALC: Target Position (Odometry) ====================
            totalAngleWrap = MathFunctions.angleWrapRad(biasAngle + turretAngleRad + robotAngleRad);
            targetTicks = totalAngleWrap * Turret.TICKS_PER_RADIAN;

            KLog.d("Turret_ODOMETRY", String.format("RAW: RobotAngle=%.2f° TurretAngle=%.2f° | CALC: Bias=%.2f° Target=%.2f° (%d ticks)",
                    Math.toDegrees(robotAngleRad), Math.toDegrees(turretAngleRad),
                    Math.toDegrees(biasAngle), Math.toDegrees(totalAngleWrap), (int) targetTicks));

        } else {
            turretMotor.setPower(0);
            isWithinRange = true;
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
            KLog.d("Turret_PID", String.format("IN_RANGE | Curr=%d Target=%d Err=%d", currentTicks, (int) targetTicks, error));
        } else {
            isWithinRange = false;
            double pidOutput = turretMotor.getPIDFController().calculate(error);
            double feedforward = TurretConfig.kF * currentAngularVelocity;
            double totalPower = Math.max(-1.0, Math.min(1.0, pidOutput + feedforward));

            KLog.d("Turret_PID", String.format("MOVING | Curr=%d Target=%d Err=%d | PID=%.3f FF=%.3f Power=%.3f",
                    currentTicks, (int) targetTicks, error, pidOutput, feedforward, totalPower));

            turretMotor.setPower(totalPower);
        }
    }

    public void runWithPower(double power) {
        this.turretRunMode = TurretRunMode.RUN_WITH_POWER;
        this.targetPower = power;
    }

    public void runWithOdometryAndLimelight() {
        this.turretRunMode = TurretRunMode.RUN_USING_ODOMETRY_AND_LIMELIGHT;
    }

    public void runWithLimelight() {
        this.turretRunMode = TurretRunMode.RUN_USING_LIMELIGHT;
    }

    public void stop() {
        this.turretRunMode = TurretRunMode.STOP;
    }

    public double getTargetTicks() {
        return targetTicks;
    }

    public void setToleranceDeg(double newToleranceDeg) {
        toleranceTicks = newToleranceDeg * Turret.TICKS_PER_DEGREE;
    }

    private void updateAngularVelocity() {
        Position currentPosition = SharedData.getOdometryWheelIMUPosition();
        double xToGoal = targetPoint.getX() - currentPosition.getX();
        double yToGoal = targetPoint.getY() - currentPosition.getY();
        double angleToGoal = Math.atan2(yToGoal, xToGoal);
        double totalAngleToGoal = MathFunctions.angleWrapRad(angleToGoal - currentPosition.getTheta());

        if (isFirstVelocityUpdate) {
            previousTotalAngle = totalAngleToGoal;
            currentAngularVelocity = 0;
            velocityTimer.reset();
            isFirstVelocityUpdate = false;
        } else {
            double deltaTime = velocityTimer.milliseconds();
            if (deltaTime > 0) {
                currentAngularVelocity = (totalAngleToGoal - previousTotalAngle) / deltaTime;
                previousTotalAngle = totalAngleToGoal;
                velocityTimer.reset();
            }
        }
    }

    public boolean getAlignWithOdometry() {
        return useOdometryAlign;
    }
}