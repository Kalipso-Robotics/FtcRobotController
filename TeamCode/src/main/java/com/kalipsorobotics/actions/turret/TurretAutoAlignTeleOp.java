package com.kalipsorobotics.actions.turret;

import static com.kalipsorobotics.actions.turret.TurretAutoAlign.computeTicksFromAngleRad;
import static com.kalipsorobotics.decode.configs.TurretConfig.LOOK_AHEAD_TIME_MS;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
;
import com.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.decode.configs.TurretConfig;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TurretAutoAlignTeleOp extends Action {
    OpModeUtilities opModeUtilities;
    AprilTagDetectionAction aprilTagDetectionAction;
    Turret turret;
    KMotor turretMotor;
    private TurretRunMode turretRunMode;
    private double targetTicks;
    private final double targetPower;
    private final double DEFAULT_TOLERANCE_TICKS = (TurretConfig.TICKS_PER_DEGREE) * 2;
    private double toleranceTicks = DEFAULT_TOLERANCE_TICKS;
    private boolean isWithinRange = false;
    private boolean aprilTagSeen = false;

    private final double searchAngleDeg = 180;
    private final boolean hasSearched = false;
    double totalAngleWrap;
    double lastLimit;
    Point targetPoint;
    private boolean useOdometryAlign;
    private double previousTotalAngle;
    private double currentAngularVelocity;
    private final ElapsedTime velocityTimer;
    private boolean isFirstVelocityUpdate;
    private final Position lastOdometryPos;
    private int ticksOffset = 0;
    private double prevTargetTicks;
    private boolean shouldReadLimelight = false;
    private double deltaAngleDeg;


    public TurretAutoAlignTeleOp(OpModeUtilities opModeUtilities, AprilTagDetectionAction aprilTagDetectionAction, Turret turret, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.aprilTagDetectionAction = aprilTagDetectionAction;
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
        this.shouldReadLimelight = true;
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
        }
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
        aprilTagDetectionAction.updateCheckDone();
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
                isWithinRange = false;
                updateAlignToTarget();
                break;
            case RUN_USING_ODOMETRY:
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
        if (TurretConfig.SHOULD_SHOOT_ON_THE_MOVE) {
            currentPos = getPredictedPos(targetPoint, currentPos, LOOK_AHEAD_TIME_MS);
        }

        double odoTargetTicks = TurretAutoAlign.calculateTargetTicks(targetPoint, currentPos, ticksOffset);

        if (turretRunMode == TurretRunMode.RUN_USING_ODOMETRY) {
            targetTicks = odoTargetTicks;
            KLog.d("TurretAutoAlignTeleOp_Odometry_Align", "Target Ticks " + targetTicks + " Current Pos: " + currentPos);
        } else if (turretRunMode == TurretRunMode.RUN_USING_LIMELIGHT) {
            lastOdometryPos.reset(currentPos);
            double limelightAngleRad;
            if (aprilTagSeen) {
                limelightAngleRad = SharedData.getLimelightRawPosition().getGoalAngleToCamRad();

                double totalTurretAngle = currentAngleRad + limelightAngleRad;

                double ticksOffsetRad = ticksOffset / TurretConfig.TICKS_PER_RADIAN;
                totalTurretAngle += ticksOffsetRad;


                totalAngleWrap = MathFunctions.angleWrapRadHysteresis(totalTurretAngle);
                double currentTargetTicks = computeTicksFromAngleRad(totalAngleWrap);

                if (shouldReadLimelight) {
                    targetTicks = currentTargetTicks;
                    prevTargetTicks = targetTicks;
                    shouldReadLimelight = false;
                }



                KLog.d("TurretAutoAlignTeleOp_Limelight_Align",
                        "currentAngleRad: " + currentAngleRad +
                                " totalTurretAngle " + totalTurretAngle +
                                " totalAngleWrap " + totalAngleWrap +
                                " targetTicks " + targetTicks +
                                " limelightAngle " + limelightAngleRad);

// already gives reverse sign from LL bc your on the left side of april tag
            } else {
                KLog.d("TurretAutoAlignTeleOp_Limelight_Align", "No April Tag Detected. Using Previous Ticks: " + prevTargetTicks);
                targetTicks = prevTargetTicks;
            }



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

        deltaAngleDeg = error / TurretConfig.TICKS_PER_DEGREE;

        if (Math.abs(error) < Math.abs(toleranceTicks)) {
            isWithinRange = true;
            shouldReadLimelight = true;
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
        KLog.d("Turret_PID", "Turret Delta Angle " + deltaAngleDeg);
    }

    public void stop() {
        KLog.d("Turret_STOP", "Teleop turret stop called");
        setTurretRunMode(TurretRunMode.STOP);
    }

    public void setTurretRunMode(TurretRunMode turretRunMode) {
        this.turretRunMode = turretRunMode;
        update();
    }

    public TurretRunMode getTurretRunMode() {
        return this.turretRunMode;
    }

    public double getTargetTicks() {
        return targetTicks;
    }

    public void setToleranceDeg(double newToleranceDeg) {
        toleranceTicks = newToleranceDeg * TurretConfig.TICKS_PER_DEGREE;
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
    public void setTicksOffset(int ticksOffset) {
        this.ticksOffset = ticksOffset;
    }

    public double getDeltaAngleDeg() {
        return deltaAngleDeg;
    }

    public static Position getPredictedPos(Point targetPoint, Position currentPos, double lookAheadTimeMS) {
        double distanceToGoal = currentPos.toPoint().distanceTo(targetPoint);
        if (TurretConfig.SHOULD_SHOOT_ON_THE_MOVE) {
            Velocity currentVelocity = SharedData.getOdometryWheelIMUVelocity();
            Position predictedPos = currentPos.predictPos(currentVelocity, lookAheadTimeMS);

            KLog.d("SOTM", "Look Ahead Time MS: " + lookAheadTimeMS +
                    " CurrentVelocity: " + currentVelocity +
                    " Delta Pos: " + predictedPos.calculateDelta(currentPos) +
                    " Delta Dist. to goal: " + (predictedPos.toPoint().distanceTo(targetPoint) - distanceToGoal)
            );
            currentPos = predictedPos;
        }
        return currentPos;
    }
}