package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
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


public class TurretAutoAlign extends Action {
    OpModeUtilities opModeUtilities;

    Turret turret;
    KMotor turretMotor;

    private double targetTicks;

    private Point targetPoint;
    private final double DEFAULT_TOLERANCE_TICKS = (Turret.TICKS_PER_DEGREE) * 1;
    private double toleranceTicks = DEFAULT_TOLERANCE_TICKS;
    private boolean isWithinRange = false;

    private AllianceColor allianceColor;

    public TurretAutoAlign(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.allianceColor = allianceColor;


        targetPoint = new Point(TurretConfig.X_INIT_SETUP_MM, TurretConfig.Y_INIT_SETUP_MM * allianceColor.getPolarity());

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

        if (!hasStarted) {
            hasStarted = true;
        }


        targetTicks = calculateTargetTicks(targetPoint);
        KLog.d("turret", "turret offset value " + TurretConfig.TICKS_INIT_OFFSET);



        KLog.d("turret_position", " ticks " + targetTicks + " motor position " + turretMotor.getCurrentPosition() + " target ticks " + targetTicks);


        if (Math.abs(turretMotor.getCurrentPosition() - targetTicks) < Math.abs(toleranceTicks)) {
            isWithinRange = true;
            turretMotor.stop();
            KLog.d("turret_position", "Within RANGE, ticks " + targetTicks + " motor position " + turretMotor.getCurrentPosition() + " target ticks " + targetTicks);

        } else {
            isWithinRange = false;
            turretMotor.goToTargetTicks((int) targetTicks);
            KLog.d("turret_position", "NOT WITHIN RANGE, ticks " + targetTicks + " motor position " + turretMotor.getCurrentPosition() + " target ticks " + targetTicks);
        }
        KLog.d("turret_in_range", "is the turret in range " + isWithinRange);

    }


    public static double calculateTargetTicks(Point targetPoint) {

        Position currentPosition = SharedData.getOdometryIMUPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();

        double yTargetGoal = targetPoint.getY() - currentY;
        double xTargetGoal = targetPoint.getX() - currentX;
        KLog.d("turret_angle_target", "target point " + targetPoint);

        double angleTargetRadian;

        angleTargetRadian = Math.atan2(yTargetGoal, xTargetGoal);

        double currentRobotAngleRadian = currentPosition.getTheta();
        double reverseTurretAngleRadian = -currentRobotAngleRadian;

        double totalTurretAngle = angleTargetRadian + reverseTurretAngleRadian;

        double totalTurretAngleWrap = MathFunctions.angleWrapRad(totalTurretAngle);

        KLog.d("turret_angle", "total turret angle " + totalTurretAngle + " total turret angle wrap " + totalTurretAngleWrap);

        double turretRotation = (totalTurretAngleWrap) / (2 * Math.PI);
        double motorRotation = turretRotation * Turret.BIG_TO_SMALL_PULLEY;
        double currentTargetTicks = Turret.TICKS_PER_ROTATION * motorRotation + TurretConfig.TICKS_INIT_OFFSET;
        return currentTargetTicks;
    }


    public double getTargetTicks() {
        return targetTicks;
    }

    public void setToleranceDeg(double newToleranceDeg) {
        toleranceTicks = newToleranceDeg * Turret.TICKS_PER_DEGREE;
    }
}
