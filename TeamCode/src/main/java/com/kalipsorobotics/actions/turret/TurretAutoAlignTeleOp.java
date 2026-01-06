package com.kalipsorobotics.actions.turret;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
;
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


public class TurretAutoAlignTeleOp extends Action {
    OpModeUtilities opModeUtilities;
    Turret turret;
    KMotor turretMotor;
    private TurretRunMode turretRunMode;
    private double targetTicks;
    private double targetPower;
    private final double DEFAULT_TOLERANCE_TICKS = (Turret.TICKS_PER_DEGREE) * 2;
    private double toleranceTicks = DEFAULT_TOLERANCE_TICKS;
    private boolean isWithinRange = false;
    private boolean aprilTagSeen = false;

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
    private Position lastOdometryPos;
    private int ticksOffset = 0;



    public TurretAutoAlignTeleOp(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.turretRunMode = TurretRunMode.STOP; // initial mode
        this.targetTicks = 0;
        this.targetPower = 0;
        lastOdometryPos = new Position(0,0,0);
        lastOdometryPos.reset(SharedData.getOdometryWheelIMUPosition());
        targetPoint = new Point(TurretConfig.X_INIT_SETUP_MM, TurretConfig.Y_INIT_SETUP_MM * allianceColor.getPolarity());
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
            KLog.d("Turret_PID", "PIDF constants " + turretMotor.getPIDFController());
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
            case RUN_USING_ODOMETRY:
                useOdometryAlign = true;
                isWithinRange = false;
                updateAlignToTarget();
                break;
        }
        KLog.d("Turret_MODE", "CURRENT RUNNING MODE " + turretRunMode);
    }

    private void updateAlignToTarget() {
        updateAngularVelocity();
        aprilTagSeen = !SharedData.getLimelightRawPosition().isEmpty();

        double currentAngleRad = turret.getCurrentAngleRad();
//        double robotAngleRad = SharedData.getOdometryWheelIMUPosition().getTheta();
//        double odoDesiredTurretAngle = MathFunctions.angleWrapRad(defaultBiasAngle -   robotAngleRad);
        Position currentPos = SharedData.getOdometryWheelIMUPosition();
        double odoTargetTicks = TurretAutoAlign.calculateTargetTicks(targetPoint, currentPos, ticksOffset);

        if (useOdometryAlign) {
            targetTicks = odoTargetTicks;
            KLog.d("Turret_", "Target Ticks " + targetTicks + " Current Pos: " + currentPos);
        } else if (aprilTagSeen) {
            lastOdometryPos.reset(currentPos);

            double limelightAngleRad = SharedData.getLimelightRawPosition().getGoalAngleToCamRad(); // already gives reverse sign from LL bc your on the left side of april tag

            // Calculate desired absolute angle (wrapped to [-π, π])
            double desiredAngleRad = MathFunctions.angleWrapRad(currentAngleRad + limelightAngleRad);
//            biasAngleCorrection = desiredAngleRad - odoDesiredTurretAngle;
            // Compute shortest angular error to avoid ±180° boundary discontinuity
            double errorRad = MathFunctions.angleWrapRad(desiredAngleRad - currentAngleRad);

            // Add error to current ticks for continuous target (no wrap-around jump)
            double currentTicks = turretMotor.getCurrentPosition();
            targetTicks = currentTicks + (errorRad * Turret.TICKS_PER_RADIAN);
            KLog.d("Turret_Limelight", "Delta Ticks Correction Odo - LL: " + (odoTargetTicks - targetTicks));
            totalAngleWrap = desiredAngleRad;

            KLog.d("Turret_Limelight", String.format("RAW: TurretPos=%.2f° LLAngle=%.2f° CurrTicks=%d | CALC: Desired=%.2f° Err=%.2f° Target=%d ticks",
                    Math.toDegrees(currentAngleRad), Math.toDegrees(limelightAngleRad), (int) currentTicks,
                    Math.toDegrees(desiredAngleRad), Math.toDegrees(errorRad), (int) targetTicks));

        } else {
            KLog.d("TurretAutoAlign_AlignToTarget", "isWithingRange: " + isWithinRange + " aprilTagSeen: " + aprilTagSeen + " useOdometryAlign " + useOdometryAlign);
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
        this.targetPower = power;
        setTurretRunMode(TurretRunMode.RUN_WITH_POWER);
    }

    public void stop() {
        KLog.d("Turret_STOP", Log.getStackTraceString(new Throwable("stop() called")));
        setTurretRunMode(TurretRunMode.STOP);
    }

    public void setTurretRunMode(TurretRunMode turretRunMode) {
        this.turretRunMode = turretRunMode;
        update();
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

    public void setUseOdometryAlign(boolean useOdometryAlign) {
        this.useOdometryAlign = useOdometryAlign;
    }

    public void incrementTicksOffset(int ticksOffset) {
        this.ticksOffset += ticksOffset;
        this.update();
    }

    public int getTicksOffset() {
        return ticksOffset;
    }
}