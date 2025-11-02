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
    private static final double FULL_SPEED_PERCENTAGE = 0.85;
    private static final double FULL_SPEED_THRESHOLD = TARGET_VELOCITY * FULL_SPEED_PERCENTAGE; // ~2642 ticks/sec
    private static final double FULL_SPEED_DURATION_SEC = 1.5;

    private ElapsedTime fullSpeedTimer;
    private boolean atFullSpeed;

    public RunIntakeUntilFullSpeed(Intake intake) {
        this.intake = intake;
        this.motor = (DcMotorEx) intake.getIntakeMotor();
        this.fullSpeedTimer = new ElapsedTime();
        this.atFullSpeed = false;
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }

        motor.setVelocity(TARGET_VELOCITY);

        double curVelocity = Math.abs(motor.getVelocity());

        // Check if at full speed
        if (curVelocity >= FULL_SPEED_THRESHOLD) {
            if (!atFullSpeed) {
                // Just reached full speed, start timer
                atFullSpeed = true;
                fullSpeedTimer.reset();
                KLog.d("intake", "Full speed reached, starting timer");
            }

            // Check if we've been at full speed long enough
            if (fullSpeedTimer.seconds() >= FULL_SPEED_DURATION_SEC) {
                isDone = true;
                KLog.d("intake", "Full speed maintained for " + FULL_SPEED_DURATION_SEC + " seconds");
            }
        } else {
            // Dropped below full speed, reset
            if (atFullSpeed) {
                KLog.d("intake", "Dropped below full speed, resetting timer");
            }
            atFullSpeed = false;
        }

        KLog.d("intake", "velocity: " + curVelocity + " / threshold: " + FULL_SPEED_THRESHOLD + " / atFullSpeed: " + atFullSpeed);
    }

}

