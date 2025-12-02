package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.math.MathFunctions;
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

    private double xInitSetupMM; //121 inches

    private double yInitSetupMM; //46inches 1000
    private final double DEFAULT_TOLERANCE_TICKS = (Turret.TICKS_PER_DEGREE)*1;
    private double toleranceTicks = DEFAULT_TOLERANCE_TICKS;
    private boolean isWithinRange = false;

    private AllianceColor allianceColor;

    public TurretAutoAlign(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.allianceColor = allianceColor;

        xInitSetupMM = TurretConfig.X_INIT_SETUP_MM;
        yInitSetupMM = TurretConfig.Y_INIT_SETUP_MM * allianceColor.getPolarity();
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

    public void incrementYInitSetupMM(double increment) {
        yInitSetupMM += increment;
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

        KLog.d("Turret_Singleton", "Encoder position " + turretMotor.getCurrentPosition());

        Position currentPosition = SharedData.getOdometryPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();

        double yTargetGoal = yInitSetupMM - currentY;
        double xTargetGoal = xInitSetupMM - currentX;
        KLog.d("turret_angle", "y init setup" + TurretConfig.Y_INIT_SETUP_MM + "current y" + currentY);

        double angleTargetRadian;
//        if (!useAprilTag) {
            // Use atan2 for proper angle calculation in all quadrants
            angleTargetRadian = Math.atan2(yTargetGoal, xTargetGoal);
            KLog.d("target_turret_angle", "target radian " + angleTargetRadian + " degrees " + Math.toDegrees(angleTargetRadian));
//        } else {
//            angleTargetRadian = SharedData.getAngleRadToGoal();
//        }

        double currentRobotAngleRadian = currentPosition.getTheta();
        double reverseTurretAngleRadian = -currentRobotAngleRadian;

        double totalTurretAngle = angleTargetRadian + reverseTurretAngleRadian;
        // Use angleWrapRad to normalize the turret angle to [-π, π]
        double totalTurretAngleWrap = MathFunctions.angleWrapRad(totalTurretAngle);

        double turretRotation = (totalTurretAngleWrap) / (2 * Math.PI);
        double motorRotation = turretRotation * Turret.BIG_TO_SMALL_PULLEY;
        targetTicks = Turret.TICKS_PER_ROTATION * motorRotation + TurretConfig.TICKS_INIT_OFFSET;
        KLog.d("turret_angle", "total turret angle " + totalTurretAngle + " total turret angle wrap " + totalTurretAngleWrap);
        KLog.d("turret", "turret offset value " + TurretConfig.TICKS_INIT_OFFSET);



        /*
        //max cable limit reached
        if (Math.abs(totalTurretAngleWrap) >= Math.PI){
            isUnwind = true; //turn on unwind mode
            targetTicks = 0;

        } else {
            //double targetTicks = (angleTargetRadian + reverseTurretAngleRadian) * ticksPerRadian);
            double turretRotation = (totalTurretAngleWrap) / (2 * Math.PI);
            double motorRotation = turretRotation * gearRatio;
            targetTicks = ticksPerRotation * motorRotation;
        }
         if (isUnwind == true){
             //finished unwind
             if (turretMotor.getCurrentPosition() > 0 - TOLERANCE_TICKS && turretMotor.getCurrentPosition() < 0 + TOLERANCE_TICKS){
                 isUnwind = false;
             } else {
                 //still unwinding (wait)
                 KLog.d("turret angle unwind", "is unwind " + isUnwind + ". not done yet. current motor position " + turretMotor.getCurrentPosition());
                 targetTicks = 0;
             }

         }*/

        KLog.d("turret_position", " ticks " + targetTicks + " motor position "+ turretMotor.getCurrentPosition() + " target ticks " + targetTicks);


        if (Math.abs(turretMotor.getCurrentPosition() - targetTicks) < Math.abs(toleranceTicks)) {
            isWithinRange = true;
            turretMotor.stop();
            KLog.d("turret_position", "Within RANGE, ticks " + targetTicks + " motor position "+ turretMotor.getCurrentPosition() + " target ticks " + targetTicks);

        } else {
            isWithinRange = false;
            turretMotor.goToTargetTicks((int) targetTicks);
            KLog.d("turret_position", "NOT WITHIN RANGE, ticks " + targetTicks + " motor position "+ turretMotor.getCurrentPosition() + " target ticks " + targetTicks);
        }
        KLog.d("turret_in_range", "is the turret in range " + isWithinRange);




//read position of robot in the field
//calculate distance from the goal
//read angle of robot
//read turret position
//calculate target angle of turret to goal
//calculate how much the turret has to turn
//move turret to target number of ticks
//

//need odometry





        //check to see if turret is aligned with the goal

    }

    public double getTargetTicks() {
        return targetTicks;
    }

    public void setToleranceDeg(double newToleranceDeg) {
        toleranceTicks = newToleranceDeg * Turret.TICKS_PER_DEGREE;
    }
}
