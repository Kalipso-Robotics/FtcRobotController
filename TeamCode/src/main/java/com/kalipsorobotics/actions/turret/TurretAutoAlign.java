package com.kalipsorobotics.actions.turret;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TurretAutoAlign extends Action {

    // you dont get odometry pos from here mini dugong :)
    Turret turret;
    DcMotor turretMotor;

    private final double ticksPerRotation = 384.5;
    private final double gearRatio = 2.69;
    private final double degreesPerRotation = 360.0;

    private final double ticksPerDegree = (ticksPerRotation * gearRatio) / degreesPerRotation;




    public TurretAutoAlign(Turret turret) {
        this.turret = turret;
        this.turretMotor = turret.turretMotor();
        this.dependentActions.add(new DoneStateAction());
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

        double x_init_setup = 3657.6;
        double y_init_setup = -3251.2;

        Position currentPosition = SharedData.getOdometryPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();


        double yTargetGoal = y_init_setup + currentY;
        double xTargetGoal = x_init_setup - currentX;

        double angleTargetRadian = Math.atan(yTargetGoal/xTargetGoal);//180 deg = 3.14 radians
        KLog.d("turret angle", "radian " + angleTargetRadian);

        double currentRobotAngleRadian = currentPosition.getTheta();
        double reverseTurretAngleRadian = -currentRobotAngleRadian;

        double turretRotation = (angleTargetRadian + reverseTurretAngleRadian) / ( 2 * Math.PI);
        double motorRotation = turretRotation * gearRatio;
        double targetTicks = ticksPerRotation * motorRotation;
        KLog.d("turret angle", "ticks " + targetTicks );
        KLog.d("turret angle", "motor position "+ turretMotor.getCurrentPosition());


        turretMotor.setTargetPosition((int) targetTicks);
        //turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);  Idk if you want this setting we have never used it unless big banana wants it :) but if you were to use it set it inside of modules.Turret
        turretMotor.setPower(0.5);





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
