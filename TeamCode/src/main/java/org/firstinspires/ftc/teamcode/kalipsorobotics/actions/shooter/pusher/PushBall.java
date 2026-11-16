package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher;


import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.RunIntakeTime;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class PushBall extends KActionSet {
    public static final String TAG = "Shooting_PushBall";

    private final KServoAutoAction openStopper;
    private final RunIntakeTime runIntakeTime;
    private final CloseStopperInstantDone closeStopperInstantDone;
    public PushBall(Stopper stopper, Intake intake) {

        KLog.d(TAG, () -> "event=create pos=" + SharedData.getOdometryWheelIMUPosition());

        IntakeStop intakeStop = new IntakeStop(intake);
        this.addAction(intakeStop);

        openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
        openStopper.setName("openStopper");
        openStopper.setDependentActions(intakeStop);
        this.addAction(openStopper);

        runIntakeTime = new RunIntakeTime(intake, IntakeConfig.shootTimeMS, IntakeConfig.intakePower);
        runIntakeTime.setName("runIntakeTime");
        runIntakeTime.setDependentActions(openStopper);
        this.addAction(runIntakeTime);

        closeStopperInstantDone = new CloseStopperInstantDone(stopper);
        closeStopperInstantDone.setName("closeStopperInstantDone");
        closeStopperInstantDone.setDependentActions(runIntakeTime);
        this.addAction(closeStopperInstantDone);
    }

    public KServoAutoAction getOpenStopper() {
        return openStopper;
    }

    @Override
    protected void afterUpdate() {
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format("name=%s openStopperDone=%s runIntakeDone=%s closeStopperDone=%s",
                    getName(), openStopper.getIsDone(), runIntakeTime.getIsDone(), closeStopperInstantDone.getIsDone()));
        }
    }
}
