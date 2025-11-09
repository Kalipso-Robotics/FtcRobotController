package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RunUntilStallAction extends Action {
    private DcMotorEx motor;
    private DcMotor regMotor;
    private double power;
    private double maxTimeoutSec;
    private int stallCount;
    private ElapsedTime timeoutTimer;

    // goBILDA 5203 series motor specifications
    private static final double RPM = 1150.0; // RPM at 12V
    private static final double TICKS_PER_REV = ((1.0 + (46.0/11.0)) * 28.0); // ((1+(46/11)) * 28) = 145.09 ticks per revolution

    // Calculate max velocity: RPM * ticks/rev / 60 sec/min
    private static final double MAX_VELOCITY_TICKS_PER_SEC = RPM * TICKS_PER_REV / 60.0;

    // Stall detection thresholds
    private static final double STALL_CURRENT_THRESHOLD_MILLIAMPS = 5000.0;
    private static final double MIN_POWER_THRESHOLD = 0.5; // only checks for motors that run with a power over 0.5
    private static final double STALL_VELOCITY_PERCENTAGE = 0.15; // Consider stalled if velocity < 15% of expected

    public RunUntilStallAction(DcMotor motor, double power, double maxTimeoutSec) {
        this.motor = (DcMotorEx) motor;
        this.regMotor = motor;
        this.power = power;
        this.maxTimeoutSec = maxTimeoutSec;
        this.stallCount = 0;
        this.timeoutTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        if (isDone) {
            KLog.d("intake", "intake done");
            return;
        }

        if (!hasStarted) {
            KLog.d("intake", "intake started");
            motor.setPower(power);
            hasStarted = true;
            timeoutTimer.reset();
            return;
        }

        KLog.d("intake", "set intake to power");
        motor.setPower(power);

        // Check timeout
        if (timeoutTimer.seconds() > maxTimeoutSec) {
            KLog.d("intake", "timeout timer " + timeoutTimer.seconds());
            regMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
            isDone = true;
        }


        double curVelocity = Math.abs(motor.getVelocity());
        double currentDraw = motor.getCurrent(CurrentUnit.MILLIAMPS);
        double expectedVelocity = Math.abs(power) * MAX_VELOCITY_TICKS_PER_SEC;
        double stallVelocityThreshold = expectedVelocity * STALL_VELOCITY_PERCENTAGE;
        KLog.d("intake", "curVelocity " + curVelocity);
        KLog.d("intake", "curCurrentDraw " + currentDraw);


        if (Math.abs(motor.getPower()) > MIN_POWER_THRESHOLD

            && currentDraw > STALL_CURRENT_THRESHOLD_MILLIAMPS
            && curVelocity < stallVelocityThreshold) {
            stallCount++;
            KLog.d("intake", "stall detected " + stallCount);

        } else {
            stallCount = 0;
            KLog.d("intake", "stall reset");

        }

        if (stallCount > 100) {
            KLog.d("intake", "identified stall " + stallCount);
            regMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
            isDone = true;
        }
    }
}
