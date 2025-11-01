package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RunUntilStallAction extends Action {
    private DcMotorEx motor;
    private double power;

    private int stallCount;
    private final double STALL_CURRENT_THRESHOLD_MILLIAMPS = 5000.0;
    private final double MIN_POWER_THRESHOLD = 0.5; // only checks for motors that run with a power over 0.5
    private double STOPPING_THRESHOLD;
    public RunUntilStallAction(DcMotor motor, double power) {
        this.motor = (DcMotorEx) motor;
        this.power = power;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        double curVelocity = motor.getVelocity();

        double currentDraw = motor.getCurrent(CurrentUnit.MILLIAMPS);

        // check if motor is drawing a high amount of current -> stalled
        if (motor.getPower() > MIN_POWER_THRESHOLD && currentDraw > STALL_CURRENT_THRESHOLD_MILLIAMPS && curVelocity < STOPPING_THRESHOLD) {
            stallCount++;
        } else {
            stallCount = 0;
        }
        if (stallCount > 10) {
            isDone = true;
        }



    }
}
