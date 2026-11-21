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

/**
 * Feeds the held balls into the shooter: stop the intake, open the stopper, run side A and side B AT THE SAME TIME
 * (each after its own delay, IntakeConfig.FEED_A_DELAY_MS / FEED_B_DELAY_MS, both 0 by default), then close the stopper.
 * Times: FEED_A_MS / FEED_B_MS. In auto, gate it like last year: pushBall.setDependentActions(driveReady, shooterReady, turretReady).
 * Note: TeleOp's ShootAllAction uses this too, so the parallel feed changes TeleOp shooting as well.
 * Pattern source: the previous PushBall chain and decode RoundTripAction gating.
 */
public class PushBall extends KActionSet {
    public static final String TAG = "Shooting_PushBall";

    private final KServoAutoAction openStopper;
    private final RunIntakeTime feedA;
    private final RunIntakeTime feedB;
    private final CloseStopper closeStopper;
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

        // A and B feed at the same time; each side waits its own delay first (both 0 = together).
        WaitAction delayA = new WaitAction(IntakeConfig.FEED_A_DELAY_MS);
        delayA.setName("feedADelay");
        delayA.setDependentActions(openStopper);
        this.addAction(delayA);

        feedA = new RunIntakeTime(sideA, IntakeConfig.FEED_A_MS, IntakeConfig.intakePower);
        feedA.setName("feedA");
        feedA.setDependentActions(delayA);
        this.addAction(feedA);

        WaitAction delayB = new WaitAction(IntakeConfig.FEED_B_DELAY_MS);
        delayB.setName("feedBDelay");
        delayB.setDependentActions(openStopper);
        this.addAction(delayB);

        feedB = new RunIntakeTime(sideB, IntakeConfig.FEED_B_MS, IntakeConfig.intakePower);
        feedB.setName("feedB");
        feedB.setDependentActions(delayB);
        this.addAction(feedB);

        closeStopper = new CloseStopper(stopper);
        closeStopper.setName("closeStopper");
        closeStopper.setDependentActions(feedA, feedB);
        this.addAction(closeStopper);
    }

    public KServoAutoAction getOpenStopper() {
        return openStopper;
    }

    /** KActionSet never sets hasStarted; RedDecideAuto reads it to drop a shoot only before the feed starts. */
    @Override
    protected void beforeUpdate() {
        super.beforeUpdate();
        hasStarted = true;
    }

    @Override
    protected void afterUpdate() {
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format("name=%s openStopperDone=%s feedADone=%s feedBDone=%s closeStopperDone=%s",
                    getName(), openStopper.getIsDone(), feedA.getIsDone(), feedB.getIsDone(), closeStopper.getIsDone()));
        }
    }
}
