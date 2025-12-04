package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.actions.cameraVision.GoalDetectionAction;
import com.kalipsorobotics.math.MathFunctions;
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

    private final double SEARCH_DEGREES = 180;
    private boolean hasSearched = false;
    double targetAngleLimelight = 0;


    public TurretAutoAlignLimelight(OpModeUtilities opModeUtilities, Turret turret, GoalDetectionAction goalDetectionAction) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.goalDetectionAction = goalDetectionAction;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.targetTicks = 0;

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

        goalDetectionAction.updateCheckDone();
        if (SharedData.getLimelightPosition().isEmpty()) {
            turretMotor.getPIDFController().setKp(TurretConfig.kP / 10);
            if (!hasSearched) {
                hasSearched = true;
                targetAngleLimelight = Math.toRadians(180);
            }
        } else {
            targetAngleLimelight = SharedData.getLimelightPosition().getAngleToGoalRad();
            hasSearched = false;
            turretMotor.getPIDFController().setKp(TurretConfig.kP);
        }
        KLog.d("Turret_Limelight", "Limelight angle" + targetAngleLimelight);
        double currentAngleRad = turret.getCurrentAngleRad();
        double totalAngleWrap = MathFunctions.angleWrapRad(currentAngleRad - targetAngleLimelight);

//        double turretRotation = (totalAngleWrap) / (2 * Math.PI);
//        double motorRotation = turretRotation * Turret.BIG_TO_SMALL_PULLEY;
//        targetTicks = Turret.TICKS_PER_ROTATION * motorRotation;

        targetTicks = totalAngleWrap * Turret.TICKS_PER_RADIAN;

        if (!hasStarted) {
            hasStarted = true;
        }

        if (Math.abs(turretMotor.getCurrentPosition() - targetTicks) < Math.abs(toleranceTicks)) {
            isWithinRange = true;
            turretMotor.stop();
            KLog.d("turret_position", String.format("Within RANGE - motor position %.2f, target ticks %.2f", (double)turretMotor.getCurrentPosition(), targetTicks));

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

}
