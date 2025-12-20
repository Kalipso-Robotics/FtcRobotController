package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Point;
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


    public TurretAutoAlignLimelight(OpModeUtilities opModeUtilities, Turret turret, GoalDetectionAction goalDetectionAction, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.goalDetectionAction = goalDetectionAction;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.targetTicks = 0;


        targetPoint = new Point(TurretConfig.X_INIT_SETUP_MM, TurretConfig.Y_INIT_SETUP_MM * allianceColor.getPolarity());
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
            runningMode = TurretRunningMode.GO_TO_NEAREST_LIMIT;
        }

        goalDetectionAction.updateCheckDone();
        aprilTagFound = !SharedData.getLimelightPosition().isEmpty();
        double targetAngleLimelight = 0;
        double currentAngleRad = turret.getCurrentAngleRad();

        if (SharedData.getLimelightPosition().isEmpty()) {
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
            KLog.d("turret_position", String.format("Within RANGE - motor position %.2f, target ticks %.2f", (double) turretMotor.getCurrentPosition(), targetTicks));

        } else {
            isWithinRange = false;
            turretMotor.goToTargetTicks((int) targetTicks);
            KLog.d("turretLL", String.format("current angle rad %.3f, target ticks %.2f, limelight angle rad %.3f", currentAngleRad, targetTicks, targetAngleLimelight));
            KLog.d("turret_position", String.format("NOT WITHIN RANGE - motor position %.2f, target ticks %.2f", (double)turretMotor.getCurrentPosition(), targetTicks));
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
}
