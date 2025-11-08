package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TurretAutoAlign extends Action {

    // you dont get odometry pos from here mini duong :)
    Turret turret;
    DcMotor turretMotor;

    private final double ticksPerRotation = 384.5;
    private final double gearRatio = 125.0/32.0;
    private final double degreesPerRotation = 360.0;
    private final double radiansPerRotation = 2 * Math.PI;

    private double xInitSetup;// = 3445.14;
    private double yInitSetup;// = 2028.8;

    public static double RED_X_INIT_SETUP = 121 *25.4; //121 inches

    public static double RED_Y_INIT_SETUP = 46 * 25.4; //46inches

    private final double TICKS_PER_RADIAN = (ticksPerRotation * gearRatio) / radiansPerRotation;
    private final double TICKS_PER_DEGREE = (ticksPerRotation * gearRatio) / degreesPerRotation;

    private final double TOLERANCE_TICKS = (TICKS_PER_DEGREE);

    private boolean isWithinRange = false;
    private boolean useAprilTag = false;


    public TurretAutoAlign(Turret turret, double xInitSetup, double yInitSetup) {
        this.turret = turret;
        this.turretMotor = turret.getTurretMotor();
        this.dependentActions.add(new DoneStateAction());
        this.xInitSetup = xInitSetup;
        this.yInitSetup = yInitSetup;
    }

    public boolean isWithinRange() {
        return isWithinRange;
    }

    public void setUseAprilTag(boolean useAprilTag) {
        this.useAprilTag = useAprilTag;
    }

    @Override
    protected void update() {
        // input: what i know
        //   x_innit_setup
        //   y_init-setup
        //   x_odo
        //   y_ooo

        //cal
        //  x_goal
        // y_goal

        //output
        //  angleTargetRadian = arctan (x_goal/y_goal)

//        double x_init_setup = 3445.14; //make static field
//        double y_init_setup = -2028.8;
       /* if(turretMotor.isBusy()){   //turn off if we need turret to be more responsive
            return;
        }*/

        Position currentPosition = SharedData.getOdometryPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();

        double yTargetGoal = yInitSetup - currentY;
        double xTargetGoal = xInitSetup - currentX;
        KLog.d("turret_angle", "y init setup" + yInitSetup + "current y" + currentY);

        double angleTargetRadian;
        if (!useAprilTag) {
            // Use atan2 for proper angle calculation in all quadrants
            angleTargetRadian = Math.atan2(yTargetGoal, xTargetGoal);
            KLog.d("target_turret_angle", "target radian " + angleTargetRadian + " degrees " + Math.toDegrees(angleTargetRadian));
        } else {
            angleTargetRadian = SharedData.getAngleRadToGoal();
        }

        double currentRobotAngleRadian = currentPosition.getTheta();
        double reverseTurretAngleRadian = -currentRobotAngleRadian;

        double totalTurretAngle = angleTargetRadian + reverseTurretAngleRadian;
        // Use angleWrapRad to normalize the turret angle to [-π, π]
        double totalTurretAngleWrap = MathFunctions.angleWrapRad(totalTurretAngle);

        double targetTicks;
        double turretRotation = (totalTurretAngleWrap) / (2 * Math.PI);
        double motorRotation = turretRotation * gearRatio;
        targetTicks = ticksPerRotation * motorRotation;
        KLog.d("turret_angle", "total turret angle " + totalTurretAngle + " total turret angle wrap " + totalTurretAngleWrap);


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


        if (turretMotor.getCurrentPosition() > targetTicks - TOLERANCE_TICKS && turretMotor.getCurrentPosition() < targetTicks + TOLERANCE_TICKS) {
            isWithinRange = true;
            SharedData.setIsTurretWithinRange(isWithinRange);
            turretMotor.setPower(0);
        } else {
            isWithinRange = false;
            SharedData.setIsTurretWithinRange(isWithinRange);
            turretMotor.setTargetPosition((int) targetTicks);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(0.7);
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
}
