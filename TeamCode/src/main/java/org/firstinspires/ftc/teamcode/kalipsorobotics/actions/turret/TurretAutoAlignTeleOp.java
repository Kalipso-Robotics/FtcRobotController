package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign.computeTicksFromAngleRad;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig.LOOK_AHEAD_TIME_MS;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.DoneStateAction;
;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTMCompensation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
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

        double odoTargetTicks = TurretAutoAlign.calculateTargetTicks(targetPoint, currentPos, ticksOffset);

        if (turretRunMode == TurretRunMode.RUN_USING_ODOMETRY) {
            targetTicks = odoTargetTicks;
            if (TurretConfig.SHOULD_SHOOT_ON_THE_MOVE) {
                double compensatedTargetHeading = computeCompensatedTargetHeading(targetPoint, currentPos, LOOK_AHEAD_TIME_MS);
                targetTicks = calculateTargetTicks(compensatedTargetHeading, currentPos.getTheta());
            }
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

    public double calculateTargetTicks(double targetAngleRad, double robotHeadingRad) {

        double reverseTurretAngleRadian = -robotHeadingRad;

        double totalTurretAngle = targetAngleRad + reverseTurretAngleRadian;

        double ticksOffsetRad = ticksOffset / TurretConfig.TICKS_PER_RADIAN;
        totalTurretAngle += ticksOffsetRad;

        double totalTurretAngleWrap = MathFunctions.angleWrapRad(totalTurretAngle);

        KLog.d("turret_angle", "total turret angle " + totalTurretAngle + " total turret angle wrap " + totalTurretAngleWrap);

        return computeTicksFromAngleRad(totalTurretAngleWrap);
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

    public static double computeCompensatedTargetHeading(Point targetPoint, Position currentPos, double lookAheadTimeMS) {
        double distanceToGoal = currentPos.toPoint().distanceTo(targetPoint);
        double targetHeadingRad = Math.atan2(targetPoint.getY() - currentPos.getY(), targetPoint.getX() - currentPos.getX());
        if (TurretConfig.SHOULD_SHOOT_ON_THE_MOVE) {
            Velocity currentVelocity = SharedData.getOdometryWheelIMUVelocity();
            Position predictedPos = currentPos.predictPos(currentVelocity, lookAheadTimeMS);
            SOTMCompensation.SOTMResult result = SOTMCompensation.calculateCompensation(targetPoint, predictedPos, currentVelocity);
            targetHeadingRad = result.getTargetAngleRad();
            KLog.d("SOTM", "Look Ahead Time MS: " + lookAheadTimeMS +
                    " CurrentVelocity: " + currentVelocity +
                    " Delta Pos: " + predictedPos.minus(currentPos) +
                    " Delta Dist. to goal: " + (predictedPos.toPoint().distanceTo(targetPoint) - distanceToGoal)
            );
        }
        return targetHeadingRad;
    }
}