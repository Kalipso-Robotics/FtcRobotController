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
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.function.DoubleSupplier;

/**
 * Feeds the held balls into the shooter: stop the intake, spin both sides against the closed stopper (PRE_FEED_MS), open
 * the stopper with them still spinning, run side A and side B AT THE SAME TIME (each after its own delay,
 * IntakeConfig.FEED_A_DELAY_MS / FEED_B_DELAY_MS, both 0 by default), then close the stopper. A side with a delay skips the pre-feed.
 * Times: FEED_A_MS / FEED_B_MS, counted from the stopper being open (openStopper done + IntakeConfig.STOPPER_OPEN_MS). In auto, gate it like last year: pushBall.setDependentActions(driveReady, shooterReady, turretReady).
 * Note: TeleOp's ShootAllAction uses this too, so the parallel feed changes TeleOp shooting as well.
 * Pattern source: the previous PushBall chain and decode RoundTripAction gating.
 */
public class PushBall extends KActionSet {
    public static final String TAG = "Shooting_PushBall";

    private final IntakeModule sideA;
    private final IntakeModule sideB;
    private final boolean preFeedA;
    private final boolean preFeedB;
    private final double feedPower = IntakeConfig.SHOOT_FEED_POWER;
    private final WaitAction preFeed;
    private final KServoAutoAction openStopper;
    private final WaitAction stopperOpen;
    private final RunIntakeTime feedA;
    private final RunIntakeTime feedB;
    private final CloseStopper closeStopper;
    public PushBall(Stopper stopper, Intake intake) {
        this(stopper, intake, null);
    }

    /**
     * shooterRpsError = shooter target RPS - actual RPS, read every loop. With IntakeConfig.FEED_RPS_COMP_ON the feed
     * speeds up when the flywheel is below target and slows when above. null = plain fixed feed speed.
     */
    public PushBall(Stopper stopper, Intake intake, DoubleSupplier shooterRpsError) {

        KLog.d(TAG, () -> "event=create pos=" + SharedData.getOdometryWheelIMUPosition());

        IntakeStop intakeStop = new IntakeStop(intake);
        this.addAction(intakeStop);

        // The wait only times the pre-feed; afterUpdate() spins the sides until the stopper is open.
        preFeed = new WaitAction(IntakeConfig.PRE_FEED_MS);
        preFeed.setName("preFeed");
        preFeed.setDependentActions(intakeStop);
        this.addAction(preFeed);

        openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
        openStopper.setName("openStopper");
        openStopper.setDependentActions(preFeed);
        this.addAction(openStopper);

        // openStopper is done on KServo's travel estimate; wait the measured open time too so the feed timers start open.
        stopperOpen = new WaitAction(IntakeConfig.STOPPER_OPEN_MS);
        stopperOpen.setName("stopperOpen");
        stopperOpen.setDependentActions(openStopper);
        this.addAction(stopperOpen);

        sideA = intake.getModule(BeamBreakPosition.IntakeSide.A);
        sideB = intake.getModule(BeamBreakPosition.IntakeSide.B);
        preFeedA = IntakeConfig.FEED_A_DELAY_MS <= 0;
        preFeedB = IntakeConfig.FEED_B_DELAY_MS <= 0;
        DoubleSupplier feedRps = shooterRpsError == null || !IntakeConfig.FEED_RPS_COMP_ON
                ? () -> IntakeConfig.FEED_RPS
                : () -> IntakeConfig.FEED_RPS <= 0 ? 0 // closed loop off: comp off too
                        : Math.max(0, IntakeConfig.FEED_RPS + IntakeConfig.FEED_RPS_COMP_GAIN * shooterRpsError.getAsDouble());

        // A and B feed at the same time; each side waits its own delay first (both 0 = together).
        WaitAction delayA = new WaitAction(IntakeConfig.FEED_A_DELAY_MS);
        delayA.setName("feedADelay");
        delayA.setDependentActions(stopperOpen);
        this.addAction(delayA);

        // Closed loop on each side's encoder when IntakeConfig.FEED_RPS > 0; the pre-feed below stays open loop.
        feedA = new RunIntakeTime(sideA, IntakeConfig.FEED_A_MS, feedPower).holdRPS(feedRps);
        feedA.setName("feedA");
        feedA.setDependentActions(delayA);
        this.addAction(feedA);

        WaitAction delayB = new WaitAction(IntakeConfig.FEED_B_DELAY_MS);
        delayB.setName("feedBDelay");
        delayB.setDependentActions(stopperOpen);
        this.addAction(delayB);

        feedB = new RunIntakeTime(sideB, IntakeConfig.FEED_B_MS, feedPower).holdRPS(feedRps);
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
        // Never zeroed at the handover: the sides keep this power until feedA / feedB take over.
        if (!stopperOpen.getIsDone()) {
            if (preFeedA) {
                sideA.run(feedPower);
            }
            if (preFeedB) {
                sideB.run(feedPower);
            }
        }
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format("name=%s preFeedDone=%s openStopperDone=%s stopperOpenDone=%s feedADone=%s feedBDone=%s closeStopperDone=%s",
                    getName(), preFeed.getIsDone(), openStopper.getIsDone(), stopperOpen.getIsDone(), feedA.getIsDone(), feedB.getIsDone(), closeStopper.getIsDone()));
        }
    }
}
