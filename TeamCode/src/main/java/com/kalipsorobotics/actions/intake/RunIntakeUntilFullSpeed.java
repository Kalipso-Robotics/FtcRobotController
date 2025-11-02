package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RunIntakeUntilFullSpeed extends Action {
    private Intake intake;
    private DcMotorEx motor;
    private static final double RPM = 1150.0; // RPM
    private static final double TICKS_PER_REV = ((1.0 + (46.0/11.0)) * 28.0); // ((1+(46/11)) * 28) = 145.09 ticks per revolution
    private static final double MAX_VELOCITY_TICKS_PER_SEC = RPM * TICKS_PER_REV / 60.0; // ~2781 ticks/sec
    private static final double TARGET_VELOCITY = MAX_VELOCITY_TICKS_PER_SEC;
    private static final double FULL_SPEED_PERCENTAGE = 0.95;
    private static final double FULL_SPEED_THRESHOLD = TARGET_VELOCITY * FULL_SPEED_PERCENTAGE; // ~2642 ticks/sec

    public RunIntakeUntilFullSpeed(Intake intake) {
        this.intake = intake;
        this.motor = (DcMotorEx) intake.getIntakeMotor();
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }

        motor.setVelocity(TARGET_VELOCITY);

        double curVelocity = Math.abs(motor.getVelocity());

        if (curVelocity >= FULL_SPEED_THRESHOLD) {
            isDone = true;
        }
        KLog.d("intake", "velocity" + curVelocity);
    }

}

