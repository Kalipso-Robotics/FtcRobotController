package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RunIntakeTime extends Action {
    private final Intake intake;
    private double timeMS;
    private final double power;
    private final ElapsedTime timer;

    public RunIntakeTime(Intake intake, double timeMS, double power) {
        this.intake = intake;
        this.timeMS = timeMS;
        this.power = power;
        timer = new ElapsedTime();
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            timer.reset();
            hasStarted = true;
        }
        if (isDone) {
            return;
        }
        intake.getIntakeMotor().setPower(power);

        if (timer.milliseconds() > timeMS) {
            KLog.d("intake", "RunIntake Timer met, stopping intake " + "current time: " + timer.milliseconds() + " ms");
            intake.getIntakeMotor().setPower(0);
            isDone = true;
        }
        KLog.d("intake", "RunIntake Timer, current time: " + timer.milliseconds() + " ms" + " threshold: " + timeMS);
    }

    public void setTimeMS(double timeMS) {
        this.timeMS = timeMS;
    }
}
