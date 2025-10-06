package com.kalipsorobotics.actions.actionUtilities.turretActions;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TurretAction extends Action {
    Odometry odometry;
    Turret turret;
    DcMotor motor;

    private final double ticksPerRotation = 384.5;
    private final double gearRatio = 2.69;
    private final double degreesPerRotation = 360.0;

    private final double ticksPerDegree = (ticksPerRotation * gearRatio) / degreesPerRotation;




    public TurretAction (Odometry odometry, Turret turret){
        this.odometry = odometry;
        this.turret = turret;
        this.motor = turret.turretMotor;
        this.dependentActions.add(new DoneStateAction());
    }

    @Override
    protected boolean checkDoneCondition() {
        return false;
        //check to see if turret is aligned with the goal

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
        //  angle_target = arctan (x_goal/y_goal)

        double x_init_setup = 3657.6;
        double y_init_setup = 3251.2;

        Position currentPosition = SharedData.getOdometryPosition();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();


        double yTargetGoal = y_init_setup + currentY;
        double xTargetGoal = x_init_setup - currentX;

        double angle_target = Math.atan(yTargetGoal/xTargetGoal);
        KLog.d("turret angle", angle_target + " ");

        double turretRotation = angle_target / 360;
        double motorRotation = turretRotation * gearRatio;
        double numberOfTicks = ticksPerRotation * motorRotation;

        motor.setTargetPosition((int) numberOfTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);








//read position of robot in the field
//calculate distance from the goal
//read angle of robot
//read turret position
//calculate target angle of turret to goal
//calculate how much the turret has to turn
//move turret to target number of ticks
//

//need odometry
    }
}
