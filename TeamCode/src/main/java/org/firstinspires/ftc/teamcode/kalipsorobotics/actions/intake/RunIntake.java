package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/** Runs both intake sides until something else stops them. Never finishes on its own. */
public class RunIntake extends Action {
    public static final String TAG = "Intaking_RunIntake";

    private final Intake intake;
    private final double power;

    public RunIntake(Intake intake) {
        this.intake = intake;
        this.power = IntakeConfig.intakePower;
    }

    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
            intake.lower(); // interim: the milestone 2 IntakeController replaces this
            KLog.d(TAG, () -> "name=" + getName() + " event=start power=" + power);
        }
        intake.run(power);
    }
}
