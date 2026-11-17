package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher;


import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.RunIntakeTime;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

// Pattern: the previous PushBall chain; feed sequence per BIOBUZZ handoff (stopper open, feed A, wait, feed B, close).
public class PushBall extends KActionSet {
    public static final String TAG = "Shooting_PushBall";

    private final KServoAutoAction openStopper;
    private final RunIntakeTime feedA;
    private final RunIntakeTime feedB;
    private final CloseStopperInstantDone closeStopperInstantDone;
    public PushBall(Stopper stopper, Intake intake) {

        KLog.d(TAG, () -> "event=create pos=" + SharedData.getOdometryWheelIMUPosition());

        IntakeStop intakeStop = new IntakeStop(intake);
        this.addAction(intakeStop);

        openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
        openStopper.setName("openStopper");
        openStopper.setDependentActions(intakeStop);
        this.addAction(openStopper);

        IntakeSide sideA = intake.getSide(BeamBreakPosition.IntakeSide.A);
        IntakeSide sideB = intake.getSide(BeamBreakPosition.IntakeSide.B);

        feedA = new RunIntakeTime(sideA, IntakeConfig.FEED_A_MS, IntakeConfig.intakePower);
        feedA.setName("feedA");
        feedA.setDependentActions(openStopper);
        this.addAction(feedA);

        WaitAction waitBetweenSides = new WaitAction(IntakeConfig.WAIT_BETWEEN_SIDES_MS);
        waitBetweenSides.setName("waitBetweenSides");
        waitBetweenSides.setDependentActions(feedA);
        this.addAction(waitBetweenSides);

        feedB = new RunIntakeTime(sideB, IntakeConfig.FEED_B_MS, IntakeConfig.intakePower);
        feedB.setName("feedB");
        feedB.setDependentActions(waitBetweenSides);
        this.addAction(feedB);

        closeStopperInstantDone = new CloseStopperInstantDone(stopper);
        closeStopperInstantDone.setName("closeStopperInstantDone");
        closeStopperInstantDone.setDependentActions(feedB);
        this.addAction(closeStopperInstantDone);
    }

    public KServoAutoAction getOpenStopper() {
        return openStopper;
    }

    @Override
    protected void afterUpdate() {
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format("name=%s openStopperDone=%s feedADone=%s feedBDone=%s closeStopperDone=%s",
                    getName(), openStopper.getIsDone(), feedA.getIsDone(), feedB.getIsDone(), closeStopperInstantDone.getIsDone()));
        }
    }
}
