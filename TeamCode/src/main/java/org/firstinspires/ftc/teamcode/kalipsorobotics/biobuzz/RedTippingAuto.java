package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.decode.DepotRoundTrip;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Tilter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

import java.util.EnumSet;
import java.util.Set;

@Autonomous
public class RedTippingAuto extends KOpMode {

    // =============================================================================================
    // THE BRAIN: decide()
    //
    // Every time the robot finishes something (a drive, a shot, a pickup) it asks decide():
    //     "Given what is going on right now, what should I do NEXT?"
    // decide() looks at a few numbers and answers with exactly ONE move from the list below.
    //
    // decide() does not touch the robot at all. It only takes numbers in and gives a move out.
    // That is why we can test it on a laptop without a robot.
    // =============================================================================================

    /** Everything decide() is allowed to answer. (WAIT is for later, when the Limelight is added.) */
    public enum Move {
        SHOOT_A,         // drive to the A launch point and shoot into Cell A
        SHOOT_B,         // drive to the B launch point and shoot into Cell B
        GET_GARDEN,      // drive to the Garden pile and pick up balls
        GET_FLOWER_A,    // drive to Flower A and pick up balls
        GET_FLOWER_B,    // drive to Flower B and pick up balls
        GET_SEEN_BALLS,  // the piles are gone, so drive to loose balls the camera sees
        WAIT,            // stay near the A launch point and hold our balls (NOT_SOLO, while Cell B is up)
        PARK             // drive to the Loading Zone and stop
    }

    /** What our partner robot is doing. This only changes which piles we visit and how many balls it helps with. */
    public enum PartnerMode {
        SOLO_NOTHING,    // partner does nothing at all
        SOLO_PRELOAD,    // partner shoots its 4 preload balls after our first tip, then parks
        NOT_SOLO         // partner owns Cell B and Flower B, we own Cell A: we shoot ONLY at A and never drive to B
    }

    /** The two Hive Cells we try to tip. */
    public enum Cell { A, B }

    /** The piles of balls we know about from the start. */
    public enum Pile { GARDEN, FLOWER_A, FLOWER_B }

    // ----- Numbers the team still has to measure. These are guesses for now. -----

    /** When this many seconds (or fewer) are left, stop collecting and go park. */
    public static final double PARK_START_SECONDS = 3;

    /**
     * Balls needed in a Cell to tip it the FIRST time. The Cell already holds 3 nectar at the start
     * (manual 10.3.1) and our robot shoots its 4 pollen preload (G304, max 4 held at once, G407).
     * 3 + 4 = 7. The manual gives NO tip weight, so this comes from the team's own test.
     */
    public static final int FIRST_TIP_BALLS = 7;

    /** Balls needed in a Cell to tip it every time after that: 8 pollen (two trips of 4). Team test, not the manual. */
    public static final int LATER_TIP_BALLS = 8;

    /** How many balls our partner shoots into a Cell for us when it shoots its preload. */
    public static final int PARTNER_PRELOAD_BALLS = 4;

    /** How many balls the Cell needs before it tips. tipsDone = how many tips already happened. */
    public static int ballsNeededToTip(int tipsDone) {
        if (tipsDone == 0) {
            return FIRST_TIP_BALLS;
        }
        return LATER_TIP_BALLS;
    }

    /**
     * How many balls our partner has ALREADY put into the new Cell that just stood up after our
     * first tip. If the partner shoots its preload, the new Cell starts with 4 balls in it, so the
     * second tip needs one trip less from us.
     */
    public static int partnerBallsInNewCell(PartnerMode mode, int tipsDone) {
        boolean noTipYet = (tipsDone == 0);
        boolean partnerDoesNothing = (mode == PartnerMode.SOLO_NOTHING);
        if (noTipYet || partnerDoesNothing) {
            return 0;
        }
        return PARTNER_PRELOAD_BALLS;
    }

    /**
     * True if a ball at this x position (mm, in our own alliance's frame) is on OUR side of the field.
     * In AUTO we never drive onto the opponent's side, so balls over there must not be counted as
     * "loose balls seen" and decide() must never be sent to get them.
     */
    public static boolean isOnOurSide(double ballXmm) {
        return ballXmm < FieldConfig.MIDLINE_X;
    }

    /** The order we visit the piles in. The partner mode decides it. */
    public static Pile[] pileOrder(PartnerMode mode) {
        if (mode == PartnerMode.NOT_SOLO) {
            // Partner takes Flower B, so we go to Flower A first and Flower B last.
            return new Pile[]{Pile.GARDEN, Pile.FLOWER_A, Pile.FLOWER_B};
        }
        return new Pile[]{Pile.GARDEN, Pile.FLOWER_B, Pile.FLOWER_A};
    }

    /**
     * The first pile in our order that still has balls, or null if no pile is left.
     * Special case: in NOT_SOLO the partner works Flower B, so we only go there
     * if the camera still sees loose balls on the field.
     */
    public static Pile nextPile(PartnerMode mode, Set<Pile> pilesLeft, int looseBallsSeen) {
        for (Pile pile : pileOrder(mode)) {
            boolean pileIsEmpty = !pilesLeft.contains(pile);
            if (pileIsEmpty) {
                continue; // nothing there, look at the next pile
            }
            boolean partnerHasThisPile = (mode == PartnerMode.NOT_SOLO && pile == Pile.FLOWER_B);
            if (partnerHasThisPile && looseBallsSeen == 0) {
                continue; // partner already took it and nothing is left to see
            }
            return pile;
        }
        return null; // no pile left
    }

    /**
     * Picks the next move. The rules are checked from top to bottom, the FIRST one that fits wins:
     *
     *   1. Almost out of time  -> PARK (parking is worth points).
     *                             Exception: if one more shot is SURE to tip a Cell, take the shot first.
     *   2. Holding balls       -> shoot them into the Cell that is standing UP.
     *                             Exception, NOT_SOLO: Cell B is the partner's, so if B is up we WAIT.
     *   3. Empty, a pile left  -> go get balls from the next pile.
     *   4. Empty, no pile left, but the camera sees loose balls -> go get those.
     *   5. Nothing left to do  -> PARK.
     *                             Exception, NOT_SOLO: if B is up we WAIT for the partner to tip it.
     *
     * In NOT_SOLO the plan is: we tip A, then the partner shoots B, then A comes up again for us.
     * We trust that agreement and never drive to B. Watching what really happens (a pickup that
     * finds nothing, the Limelight seeing which Cell is up) is how we cope when the partner differs.
     *
     * @param mode            what our partner robot is doing
     * @param secondsLeft     seconds left in AUTO
     * @param ballsHeld       balls inside our robot right now (from the beam break)
     * @param atShootPoint    true if the robot is already standing at the launch point of the UP Cell
     * @param upCell          the Cell that is standing UP (the one we want to tip)
     * @param ballsInUpCell   balls already inside that Cell (ours plus the partner's)
     * @param tipsDone        how many tips already happened
     * @param pilesLeft       the piles that still have balls (remove a pile when a pickup found nothing)
     * @param looseBallsSeen  loose balls the camera sees on OUR side of the field only. Never count a
     *                        ball on the opponent's side, because we do not cross over in AUTO
     *                        (see isOnOurSide)
     */
    public static Move decide(PartnerMode mode, double secondsLeft, int ballsHeld, boolean atShootPoint,
                              Cell upCell, int ballsInUpCell, int tipsDone,
                              Set<Pile> pilesLeft, int looseBallsSeen) {

        boolean holdingBalls = (ballsHeld > 0);

        // The shot we would take: into whichever Cell is standing up.
        Move shootUpCell;
        if (upCell == Cell.A) {
            shootUpCell = Move.SHOOT_A;
        } else {
            shootUpCell = Move.SHOOT_B;
        }

        // In NOT_SOLO the partner owns Cell B. If B is the one standing up, it is not our turn.
        boolean notOurTurn = (mode == PartnerMode.NOT_SOLO && upCell == Cell.B);

        // Rule 1: almost out of time.
        boolean almostOutOfTime = (secondsLeft <= PARK_START_SECONDS);
        if (almostOutOfTime) {
            boolean thisShotWillTip = (ballsInUpCell + ballsHeld >= ballsNeededToTip(tipsDone));
            boolean sureTip = holdingBalls && atShootPoint && thisShotWillTip && !notOurTurn;
            if (sureTip) {
                return shootUpCell; // one last shot that tips the Cell, then the time runs out
            }
            return Move.PARK;
        }

        // Rule 2: we have balls, so shoot them (unless it is the partner's turn: then wait).
        if (holdingBalls) {
            if (notOurTurn) {
                return Move.WAIT;
            }
            return shootUpCell;
        }

        // Rule 3: we are empty, so get balls from the next pile that still has some.
        Pile pile = nextPile(mode, pilesLeft, looseBallsSeen);
        if (pile == Pile.GARDEN) {
            return Move.GET_GARDEN;
        }
        if (pile == Pile.FLOWER_A) {
            return Move.GET_FLOWER_A;
        }
        if (pile == Pile.FLOWER_B) {
            return Move.GET_FLOWER_B;
        }

        // Rule 4: no pile left, but the camera still sees balls lying around.
        if (looseBallsSeen > 0) {
            return Move.GET_SEEN_BALLS;
        }

        // Rule 5: nothing left to do. In NOT_SOLO, if B is up we wait for the partner's tip.
        if (notOurTurn) {
            return Move.WAIT;
        }
        return Move.PARK;
    }

    // =============================================================================================
    // THE FAKE SCENARIO (for testing on the Driver Hub before the robot can sense anything)
    //
    // decide() needs inputs. The real robot cannot give them yet (no beam break, no camera, no
    // shooting). So for now we type the inputs here by hand. These are the "pretend" situation.
    // To test another situation: change a number below, then deploy and run again.
    // =============================================================================================
    private static final double AUTO_SECONDS = 30;       // how long AUTO lasts; seconds left = this minus run time
    private static final PartnerMode FAKE_PARTNER_MODE = PartnerMode.SOLO_NOTHING;
    private int fakeBallsHeld = 0;                       // pretend we hold this many balls (rule G407: at most 4)
    private Cell fakeUpCell = Cell.A;                    // pretend this Cell is standing up
    private int fakeBallsInUpCell = 3;                   // the up Cell starts with 3 nectar (manual 10.3.1)
    private int fakeTipsDone = 0;                        // pretend this many tips already happened
    private int fakeLooseBallsSeen = 0;                  // pretend the camera sees this many loose balls
    private Set<Pile> fakePilesLeft = EnumSet.allOf(Pile.class); // pretend all three piles still have balls

    /** The move decide() gave last time, so we only write to logcat when the answer CHANGES. */
    private Move previousMove = null;

    /**
     * Asks decide() what to do, using the fake scenario above, and SHOWS the question and the answer:
     *   - on the Driver Hub screen (telemetry), refreshed every call
     *   - in logcat (search for "KLog_Decide"), only when the answer changes
     * It does NOT drive the robot. It only shows what decide() would say.
     */
    private Move decideAndShow(double secondsLeft, boolean atShootPoint) {
        Move move = decide(FAKE_PARTNER_MODE, secondsLeft, fakeBallsHeld, atShootPoint, fakeUpCell,
                fakeBallsInUpCell, fakeTipsDone, fakePilesLeft, fakeLooseBallsSeen);

        // Driver Hub screen: the answer first, then every number it was based on.
        telemetry.addData("decide says", move);
        telemetry.addData("partner mode", FAKE_PARTNER_MODE);
        telemetry.addData("seconds left", secondsLeft);
        telemetry.addData("balls held", fakeBallsHeld);
        telemetry.addData("at shoot point", atShootPoint);
        telemetry.addData("up cell", fakeUpCell + " (" + fakeBallsInUpCell + " balls inside)");
        telemetry.addData("tips done", fakeTipsDone);
        telemetry.addData("piles left", fakePilesLeft);
        telemetry.addData("loose balls seen", fakeLooseBallsSeen);
        telemetry.update();

        // Logcat: one line with the same information, but only when the answer changes.
        boolean answerChanged = (move != previousMove);
        if (answerChanged) {
            KLog.d("Decide", () -> "partner=" + FAKE_PARTNER_MODE + " secondsLeft=" + secondsLeft
                    + " ballsHeld=" + fakeBallsHeld + " atShootPoint=" + atShootPoint
                    + " upCell=" + fakeUpCell + " ballsInUpCell=" + fakeBallsInUpCell
                    + " tipsDone=" + fakeTipsDone + " pilesLeft=" + fakePilesLeft
                    + " looseBallsSeen=" + fakeLooseBallsSeen + " -> " + move);
            previousMove = move;
        }
        return move;
    }

    KActionSet autoTipping;

    private DriveTrain driveTrain;
//    Shooter shooter = null;
//    Intake intake = null;
//    Stopper stopper = null;
//    Turret turret = null;
//
//    TurretAutoAlign turretAutoAlign = null;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // Optional: let hardware initialize

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 171.5 + 1218, 3658 - 171.5, Math.toRadians(-90));
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        autoTipping = new KActionSet();
        KLog.d("RedAutoDepot-Init", "Creating intake, shooter, stopper modules");
        KLog.d("RedAutoDepot-Init", () -> "opModeUtilities is: " + (opModeUtilities != null ? "NOT NULL" : "NULL"));
//        intake = new Intake(opModeUtilities);
//        shooter = new Shooter(opModeUtilities);
//        stopper = new Stopper(opModeUtilities);
//        shooterRun = new ShooterRun(opModeUtilities, shooter, 0, ShooterInterpolationConfig.MAX_HOOD);
//        shooterRun.setShooterRunMode(ShooterRunMode.STOP);
//        KLog.d("RedAutoDepot-Init", () -> "Stopper created: " + (stopper != null ? "SUCCESS" : "NULL"));
//
//        Turret.setInstanceNull();
//        turret = Turret.getInstance(opModeUtilities);
//        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        // A shoot, go to garden
        RoundTripAction trip1 = new RoundTripAction(opModeUtilities, driveTrain, /* turretAutoAlign, shooter, stopper, intake, */ allianceColor, true);
        trip1.getMoveToShoot().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), -90);
        trip1.getMoveToBalls().addPoint(FieldConfig.aLaunchPoint.getX(),FieldConfig.aLaunchPoint.getY(),-180);
        trip1.getMoveToBalls().addPoint(600,FieldConfig.aLaunchPoint.getY(),-180);
        trip1.getMoveToBalls().addPoint(400,FieldConfig.aLaunchPoint.getY() - 200,90);
        trip1.getMoveToBalls().addPoint(FieldConfig.gardenPoint.getX(),FieldConfig.gardenPoint.getY() - 200,90);
//        trip1.getMoveToBalls().addPoint(FieldConfig.gardenPoint.getX(),FieldConfig.gardenPoint.getY(),90);
        trip1.setName("trip1");
        autoTipping.addAction(trip1);

        // B shoot, go to B flower
        RoundTripAction trip2 = new RoundTripAction(opModeUtilities, driveTrain, /* turretAutoAlign, shooter, stopper, intake, */ allianceColor, true);
        trip2.getMoveToShoot().addPoint(600, 3658-600, 0);
        trip2.getMoveToShoot().addPoint(1500, 3658-600, 90);
//        trip2.getMoveToShoot().addPoint(1300, 600, -90);
        trip2.getMoveToShoot().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), 90);
        trip2.getMoveToBalls().addPoint(FieldConfig.bLaunchPoint.getX(), 700, -180);
        trip2.getMoveToBalls().addPoint(FieldConfig.bFlowerPoint.getX(),FieldConfig.bFlowerPoint.getY() + 500,-90);
        trip2.getMoveToBalls().addPoint(FieldConfig.bFlowerPoint.getX(),FieldConfig.bFlowerPoint.getY() + 300,-90);
        trip2.setName("trip2");
        trip2.setDependentActions(trip1);
        autoTipping.addAction(trip2);

        // A shoot, go to A flower
        RoundTripAction trip3 = new RoundTripAction(opModeUtilities, driveTrain, /* turretAutoAlign, shooter, stopper, intake, */ allianceColor, true);
        trip3.getMoveToShoot().addPoint(900, 600, -90);
        trip3.getMoveToShoot().addPoint(900, 2700, -90);
        trip3.getMoveToShoot().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), -90);
        trip3.getMoveToBalls().addPoint(FieldConfig.aFlowerPoint.getX() + 200,FieldConfig.aFlowerPoint.getY(),180);
        trip3.setName("trip3");
        trip3.setDependentActions(trip2);
        autoTipping.addAction(trip3);

        // B shoot, stay B side
        RoundTripAction trip4 = new RoundTripAction(opModeUtilities, driveTrain, /* turretAutoAlign, shooter, stopper, intake, */ allianceColor, true);
        trip4.getMoveToShoot().addPoint(600, 3658-600, 0);
        trip4.getMoveToShoot().addPoint(1500, 3658-600, 90);
        trip4.getMoveToShoot().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), 90);
        //trip4.getMoveToBalls().addPoint(0,0,0);
        trip4.setName("trip4");
        trip4.setDependentActions(trip3);
        autoTipping.addAction(trip4);

        // A shoot, go to A side
        RoundTripAction trip5 = new RoundTripAction(opModeUtilities, driveTrain, /* turretAutoAlign, shooter, stopper, intake, */ allianceColor, true);
        trip5.getMoveToShoot().addPoint(900, 600, -90);
        trip5.getMoveToShoot().addPoint(900, 2700, -90);
        trip5.getMoveToShoot().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), -90);
//        trip5.getMoveToBalls().addPoint(0,0,0);
        trip5.setName("trip5");
        trip5.setDependentActions(trip4);
        autoTipping.addAction(trip5);

        // B shoot
        RoundTripAction trip6 = new RoundTripAction(opModeUtilities, driveTrain, /* turretAutoAlign, shooter, stopper, intake, */ allianceColor, true);
        trip6.getMoveToShoot().addPoint(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), 90);
        trip6.getMoveToShoot().addPoint(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), 90);
        trip6.setName("trip6");
        //autoTipping.addAction(trip6);

        IPurePursuitAction park = new AdaptivePurePursuitAction(driveTrain);
        park.addPoint(1000, 1000, 180);
        park.addPoint(100, 3657.6-3350-200, 180);
        park.setName("park");
        park.setDependentActions(trip6);
        autoTipping.addAction(park);

        waitForStartPrecomputingPaths();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            autoTipping.updateCheckDone();
            // Only SHOWS what decide() would say (fake scenario above). It does not move the robot yet.
            decideAndShow(AUTO_SECONDS - getRuntime(), false);
            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        cleanupRobot();
    }
}
