package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

public class IntakeFullAction extends KActionSet {
    public static final String TAG = "Intaking_IntakeFullAction";

    private final RunIntakeTime runIntakeTime;

    private final Intake intake; // null when one side runs
    private final IntakeSide side; // null when both sides run

    /** Both sides. */
    public IntakeFullAction(Stopper stopper, Intake intake, double timeMS, double power) {
        this(stopper, intake, null, new RunIntakeTime(intake, timeMS, power));
    }

    /** One side only (pattern: PushBall's per-side feed). Lowers that side when it starts. */
    public IntakeFullAction(Stopper stopper, IntakeSide side, double timeMS, double power) {
        this(stopper, null, side, new RunIntakeTime(side, timeMS, power, true));
    }

    private IntakeFullAction(Stopper stopper, Intake intake, IntakeSide side, RunIntakeTime runIntakeTime) {
        this.intake = intake;
        this.side = side;

        KServoAutoAction closeStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        closeStopper.setName("closeStopper");
        this.addAction(closeStopper);

        this.runIntakeTime = runIntakeTime;
        runIntakeTime.setName("intakeRun");
        this.addAction(runIntakeTime);

    }

    public RunIntakeTime getRunIntakeTime() {
        return runIntakeTime;
    }

    public void stopAndSetDone() {
        if (isDone) {
            return;
        }
        this.setIsDone(true);
<<<<<<< HEAD
        intake.getIntakeMotorFlower().setPower(0);
=======
        if (side != null) {
            side.stop();
        } else {
            intake.stop();
        }
        KLog.d(TAG, () -> "name=" + getName() + " event=stopAndSetDone");
>>>>>>> 9c27d2d1b179e33b4e0870dc56c7d8a16e21afc9
    }
}


