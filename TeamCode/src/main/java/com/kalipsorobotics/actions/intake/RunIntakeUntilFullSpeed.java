package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RunIntakeUntilFullSpeed extends Action {
    private Intake intake;
    private DcMotorEx motor;
    private static final double RPM = 1150.0; // RPM
    private static final double TICKS_PER_REV = ((1.0 + (46.0/11.0)) * 28.0); // ((1+(46/11)) * 28) = 145.09 ticks per revolution
    private static final double MAX_VELOCITY_TICKS_PER_SEC = RPM * TICKS_PER_REV / 60.0; // ~2781 ticks/sec
    private static final double TARGET_VELOCITY = MAX_VELOCITY_TICKS_PER_SEC;
    //01-01 21:31:51.020  1576  1748 D KLog_intake: velocity: 1920.0 / threshold: 1390.4545454545455 / atFullSpeed: true
    //01-01 21:31:30.735  1576  1748 D KLog_intake: velocity: 1460.0 / threshold: 1390.4545454545455 / atFullSpeed: true
    private static final double FULL_SPEED_THRESHOLD = TARGET_VELOCITY * 0.70; // ~2642 ticks/sec / observed full speed velocity ~2000 ticks / sec, low velocity 500-900
    private double fullSpeedDurationMS = 150;

    private ElapsedTime fullSpeedTimer;
    private boolean fullSpeedTimerReset;

    public RunIntakeUntilFullSpeed(Intake intake) {
        this.intake = intake;
        this.motor = (DcMotorEx) intake.getIntakeMotor();
        this.fullSpeedTimer = new ElapsedTime();
        this.fullSpeedTimerReset = false;
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            fullSpeedTimer.reset();
            hasStarted = true;
        }

        motor.setPower(1);

        double curVelocity = Math.abs(motor.getVelocity());

        if (curVelocity > FULL_SPEED_THRESHOLD) {
            if (!fullSpeedTimerReset) {
                // timer start
                fullSpeedTimerReset = true;
                fullSpeedTimer.reset();
                KLog.d("intake", "Full speed reached, starting timer");
            } else if (fullSpeedTimer.milliseconds() > fullSpeedDurationMS) {
                isDone = true;
                KLog.d("intake", "Full speed maintained for " + fullSpeedDurationMS + " seconds");
            }
        } else {
            // reset timer
            KLog.d("intake", "Dropped below full speed, resetting timer");
            fullSpeedTimer.reset();
            fullSpeedTimerReset = false;
        }

        KLog.d("intake", "velocity: " + curVelocity + " / threshold: " + FULL_SPEED_THRESHOLD + " / atFullSpeed: " + fullSpeedTimerReset);
    }

    public void setFullSpeedDurationMs(double fullSpeedDurationMs) {
        fullSpeedDurationMS = fullSpeedDurationMs;
    }


}

