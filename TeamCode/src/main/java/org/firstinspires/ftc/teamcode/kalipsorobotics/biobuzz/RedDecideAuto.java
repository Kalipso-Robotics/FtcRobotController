package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.AutoToBallsPathingAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.beambreak.BeamBreakCountAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.DriveDirectionIntakeController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReadyAuto;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Move;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.PartnerMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Pile;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PathPlanning;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

/**
 * The AUTO where AutoDecider.decide() really drives the robot.
 *
 * The idea, in four steps that repeat until time runs out:
 *   1. We start holding the 4 preload balls, so the first answer is "shoot the up Cell".
 *   2. When the current move is DONE we update what we know (balls held, balls in each Cell, tips, piles left).
 *   3. We ask decide() for the NEXT move. A few small rules in chooseMove() then adjust the answer.
 *   4. We start that move. A move is one of: SHOOT a Cell, GET a pile, GET balls the camera saw, SWEEP a spill, PARK.
 *
 * Driving goes through the path planner (PathPlanning): it goes around the Hive legs and the flowers and stays on our side.
 *
 * Every number marked GUESS is unmeasured.
 */
@Autonomous(name = "Red Decide Auto", group = "biobuzz")
public class RedDecideAuto extends KOpMode {

    // =============================================================================================
    // Settings
    // =============================================================================================

    // ----- Rules of the game -----
    private static final double AUTO_SECONDS = 30;                 // manual 10.4
    private static final PartnerMode PARTNER_MODE = PartnerMode.SOLO_NOTHING;
    private static final int PRELOAD_BALLS = 4;                    // manual G304
    private static final int BALLS_PER_PILE = 4;                   // manual 10.3.1
    private static final int MOST_BALLS_WE_CAN_HOLD = 4;           // manual G407
    private static final int NECTAR_IN_CELL_A_AT_START = 3;        // manual 10.3.1

    // ----- How long things take -----
    private static final double SPEED_MM_PER_SECOND = 1000;        // GUESS: how fast the robot drives
    public static final String TAG = "Auto_Decider";

    private static final double PARK_WHEN_SECONDS_LEFT = 2.5;     // we drive to the park spot when this much time is left, however far it is
    private static final double SECONDS_TO_SHOOT = 2;              // GUESS
    private static final double HIVE_TIP_SECONDS = 3.3;            // GUESS from the sim: a Hive finishes tipping this long after the shot that tips it
    private static final double WAIT_MS = 500;                     // how long one WAIT lasts
    private static final double COUNT_SETTLE_SECONDS = 0.4;        // after a pickup the beam-break count lags a moment: wait before deciding
    private static final double PICKUP_GIVE_UP_SECONDS = 6;        // a pickup not finished after this long is stuck (for example against a Flower)

    // ----- Where things are -----
    private static final double GARDEN_BALLS_CENTER_X_MM = 142;    // GUESS from the sim field: the 4 Garden balls sit at x = 36 to 249
    private static final double OBSTACLE_BUFFER_MM = 262.5;        // same as DecideSimulation / AutoToBallsPathingAction
    private static final double PARK_WALL_GAP_MM = 15;             // GUESS: room left between the body and the walls at the park spot

    // ----- Shooting -----
    private static final double SHOOT_ZONE_MM = 250;               // GUESS: we shoot once the robot is this close to the launch point, even while still driving
    private static final double AT_SHOOT_POINT_MM = 150;           // closer than this to the launch point = "at the shoot point"

    // ----- Seeing and remembering balls -----
    private static final long BALL_MEMORY_MS = 3000;               // a ball seen while driving is remembered this long (spilled balls roll away). Was 5000.
    private static final double BALL_MERGE_MM = 100;               // a new sighting this close to a remembered ball is the same ball
    private static final double BALL_FORGET_NEAR_ROBOT_MM = 250;   // we drove over this spot: the ball was picked up (or the camera cannot see it)
    private static final double BALL_NEAR_CENTER_MM = 100;         // a cluster's middle counts as "on a ball" when one is this close
    private static final double LOAD_GROUP_MM = 250;               // remembered balls closer than this chain into one group
    private static final double VISITED_REPEAT_MM = 300;           // balls this close to a place we already went to are ignored

    // ----- Choosing which group of balls to go for -----
    private static final double LOAD_STOP_SECONDS = 1.0;           // GUESS: slowing down, picking up and speeding up again at one stop
    private static final double LOAD_PARTIAL_VALUE = 0.4;          // a load that does not finish the tip is worth this fraction of its progress
    private static final double COMMIT_MAX_SECONDS = 8;            // a chosen group is kept this long (the target does not change every decision)
    private static final double COMMIT_REACHED_MM = 250;           // within this of the chosen group = we are there
    private static final double COMMIT_SWITCH_SCORE = 1.2;         // a new load must score this many times the chosen one to replace it

    // ----- The viewpoint: from the third tip on, we stand here to look at every ball on the up Cell's end, then pick them up -----
    // We stand at the middle edge of our half, next to the Hive, and face the MIDDLE of the up end's area (our half, from the Hive's
    // middle to the wall), so the camera sees both sides of it. GUESSES: tune from the sim.
    private static final Point VIEWPOINT_HIGH_Y_END = new Point(800, 2000);
    private static final Point VIEW_TOWARD_HIGH_Y_END = new Point(915, 2750);  // the middle of the high-y end's area
    private static final Point VIEWPOINT_LOW_Y_END = new Point(800, 1658);
    private static final Point VIEW_TOWARD_LOW_Y_END = new Point(915, 908);    // the middle of the low-y end's area
    private static final double VIEWPOINT_REACHED_MM = 250;        // within this of the viewpoint = we are there
    private static final double LOOK_SETTLE_SECONDS = 0.7;         // after arriving, let the camera look before deciding

    private static final boolean SWEEP_ENABLED = false;            // false = never drive the sweep loop (the code stays, it is just never chosen)

    // ----- The sweep: a rectangle loop along the wall where a tipped Cell dumped its balls -----
    // Out along one lane, back along another, with the intake running. All GUESSES: tune from the sim.
    private static final double SWEEP_OUT_X_MM = 750;              // out lane, along the Hive's side of the corridor
    private static final double SWEEP_BACK_X_MM = 400;             // back lane, along the wall, just outside the flower buffer (127 + 262.5)
    private static final double SWEEP_FAR_WALL_GAP_MM = 450;       // the far end of the loop stops this far from the spill wall
    private static final double SWEEP_NEAR_INSET_MM = 450;         // the near end is this far inward from the up Cell's launch point
    private static final double SWEEP_DETOUR_MM = 400;             // a seen group this close to the leg we are driving is worth a detour
    private static final double SWEEP_CORNER_REACHED_MM = 300;     // passing this close to a corner counts the corner as done
    private static final double SWEEP_MIN_SECONDS_LEFT = 5;        // with less time than this left, no new sweep: park
    private static final long CHECK_EVERY_MS = 250;                // how often the sweep and the park-time check look around

    // =============================================================================================
    // What we know
    // =============================================================================================

    private int ballsHeld = PRELOAD_BALLS;
    Cell upCell = Cell.A;                                 // (no 'private' on the next few: RedDecideAutoPartnerFirst/NotSolo, in this package, change the starting state)
    int ballsInCellA = NECTAR_IN_CELL_A_AT_START;
    int ballsInCellB = 0;
    int tipsDone = 0;
    final Set<Pile> pilesLeft = EnumSet.allOf(Pile.class);
    final List<Cell> pendingSpills = new ArrayList<>();   // Cells that tipped and whose spilled balls we have not swept yet
    double hiveSettledAtSeconds = 0;                      // we never fire before this run time: a Hive that is still tipping would send the balls into the wrong Cell
    double decideNotBeforeSeconds = 0;                    // do not ask decide() before this run time (the ball count is still settling)

    // ----- Balls the camera saw -----
    // Two lists that go together: rememberedBalls.get(i) was last seen at rememberedAtMs.get(i).
    private final List<BallInformation> rememberedBalls = new ArrayList<>();
    private final List<Long> rememberedAtMs = new ArrayList<>();
    private long lastSightingListMs = 0;                  // the camera list we already read
    private final List<Point> visitedPlaces = new ArrayList<>(); // where we already went for seen balls (worked or not): never go back
    private BallCluster seenCluster = null;               // the first stop of the best load the camera knows about right now (null = none)
    private Point committedCenter = null;                 // the group we chose and keep until picked up, gone or clearly beaten
    private List<BallInformation> committedBalls = new ArrayList<>();
    private double committedAtSeconds = 0;

    // ----- The move that is running -----
    private Move currentMove = null;
    private Action currentAction = null;                  // null = nothing is running, so it is time to decide
    private Pile currentPile = null;                      // for GET_GARDEN / GET_FLOWER_*
    private Point currentSeenCenter = null;               // for GET_SEEN_BALLS: where it goes
    private int currentSeenSize = 0;                      //                     and how many balls it expects
    private TurretAutoAlign currentAim = null;            // for SHOOT_*: updated every loop, because the turret only moves when updated
    private double moveStartedAtSeconds = 0;
    private boolean parked = false;

    // ----- The viewpoint (while we drive to it, currentMove is WAIT) -----
    private boolean goingToViewpoint = false;             // the running move is the drive to the viewpoint
    private int viewedAtTipsDone = -1;                    // tipsDone when we last went to the viewpoint: only once per tip

    // ----- The sweep (while a sweep is running, currentMove is WAIT) -----
    private Cell sweepRunningFor = null;                  // the spill the running sweep leg is for (null = no sweep leg is running)
    private Cell sweepCell = null;                        // the spill the whole loop is for (null = no loop started)
    private int sweepLeg = 0;                             // the next corner of the loop to drive to (0-3)
    private DriveDirectionIntakeController sweepIntake = null; // runs the side facing the drive while a sweep leg drives
    private boolean sweepGivenUp = false;                 // a sweep was dropped for time: never start another
    private long lastSweepCheckMs = 0;                   // when the sweep last looked for a group beside the lane
    private long lastParkCheckMs = 0;                     // when we last checked if it is time to park

    // ----- Robot -----
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Turret turret;
    private BeamBreakCountAction beamBreaks;
    private PathPlanning planner;

    // =============================================================================================
    // Hooks: RedDecideAutoPartnerFirst and RedDecideAutoNotSolo override these and nothing else
    // =============================================================================================

    /** Where the robot starts (the middle of the robot, mm). */
    Point startPoint() {
        return new Point(171.5 + 1218, 3658 - RobotFootprint.BACK_MM); // the back touches the wall
    }

    /** Which way the front points at the start, in radians. */
    double startHeadingRad() {
        return Math.toRadians(-90);
    }

    /** The point the turret is aimed at before the first shot. */
    Point firstTurretTarget() {
        return FieldConfig.aTargetPoint;
    }

    /** Called once, right after START. Here a subclass can change what we believe about the Hive. */
    void afterStart() {
    }

    /** Lets a subclass change decide()'s answer (for example: never shoot into the partner's Cell). Normally it changes nothing. */
    Move limitMove(Move move) {
        return move;
    }

    /** Do we sweep the balls a tipped Cell dumped? A subclass says no when the partner works that end. */
    boolean ownsSpillOf(Cell tipped) {
        return true;
    }

    /**
     * Is this loose ball ours to pick up? Every ball on our side of the field. While we work toward the third tip (tipsDone is 2),
     * only balls on the same end of the Hive as the up Cell (the end we shoot from), so we do not drive to the far end and back.
     */
    boolean wantsBall(BallInformation ball) {
        if (!RedTippingAuto.isOnOurSide(ball.x)) {
            return false;
        }
        if (!staysOnUpSide()) {
            return true;
        }
        boolean ballIsOnHighYEnd = ball.y > FieldConfig.MIDLINE_X; // the field is square, so its middle is y = MIDLINE_X too
        return upCellIsOnHighYEnd() == ballIsOnHighYEnd;
    }

    /**
     * While we work toward the third tip (tipsDone is exactly 2) we stay on the end of the Hive where the up Cell is, so we do not
     * drive to the far end and back. Once the third tip is done we may cross sides again.
     */
    private boolean staysOnUpSide() {
        return tipsDone == 2;
    }

    /** True when the up Cell is on the high-y end of the field, false when it is on the low-y end. */
    private boolean upCellIsOnHighYEnd() {
        return launchPointOf(upCell).getY() > FieldConfig.MIDLINE_X;
    }

    /** Where to stand to see every ball on the up Cell's end. */
    private Point viewpointOfUpEnd() {
        return upCellIsOnHighYEnd() ? VIEWPOINT_HIGH_Y_END : VIEWPOINT_LOW_Y_END;
    }

    /** The middle of the up end's area: at the viewpoint the front (and the camera) faces it. */
    private Point lookTowardOfUpEnd() {
        return upCellIsOnHighYEnd() ? VIEW_TOWARD_HIGH_Y_END : VIEW_TOWARD_LOW_Y_END;
    }

    // =============================================================================================
    // Setup and the main loop
    // =============================================================================================

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TRIM_DEG = 0;
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // let hardware initialize

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, startPoint().getX(), startPoint().getY(), startHeadingRad());
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);

        AutoToBallsPathingAction.checkFlowerClearance();
        beamBreaks = new BeamBreakCountAction(opModeUtilities); // counts the balls inside the robot

        // AUTO: the opponent's half is blocked. Nothing is tipping yet, so the space under the Hive is open.
        planner = PathPlanning.forFieldAuto(OBSTACLE_BUFFER_MM, RobotFootprint.WIDTH_MM / 2, false);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStartPrecomputingPaths();
        afterStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();

            readBallCount();
            if (currentAim != null) {
                currentAim.updateCheckDone(); // the turret only moves when its action is updated
            }

            rememberSightings();
            parkIfOutOfTime();
            giveUpIfPickupStuck();
            checkSweep();

            if (currentAction != null && currentAction.updateCheckDone()) {
                finishMove();
            }

            // A pickup builds its drive on its first update. Finish that path's precompute now, or the robot
            // stands still while it plans one small step per loop.
            if (!AdaptivePurePursuitAction.allPrecomputeDone()) {
                precomputePathsNow();
            }

            if (currentAction == null && !parked && getRuntime() >= decideNotBeforeSeconds) {
                startNextMove();
            }

            intake.update(); // finishes side raises that waited for the Flower arm

            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        stowIntake("autoEnd");
        cleanupRobot();
    }

    /**
     * Keeps the ball count fresh from the beam breaks. While we shoot the count falls, so it is not read then:
     * we need the count from just BEFORE the shot to know how many balls went into the Cell.
     */
    private void readBallCount() {
        beamBreaks.updateCheckDone();
        boolean shooting = isShoot(currentMove) && currentAction != null;
        if (!shooting && beamBreakIsTrustworthy()) {
            ballsHeld = SharedData.getBeamBreakData().getCount();
        }
    }

    private static boolean beamBreakIsTrustworthy() {
        BeamBreakData beam = SharedData.getBeamBreakData();
        return beam.hasData() && beam.isFresh(250) && beam.isReliable();
    }

    // =============================================================================================
    // Choosing the next move
    // =============================================================================================

    private void startNextMove() {
        moveStartedAtSeconds = getRuntime();
        Point robot = robotPoint();
        double secondsLeft = AUTO_SECONDS - getRuntime();

        Move move = chooseMove(robot, secondsLeft);

        telemetry.addData("decide says", move);
        telemetry.update();

        startMove(move);
        precomputePathsNow();
    }

    /**
     * Asks decide(), then adjusts its answer with the "full load" rules. Sets sweepRunningFor when the answer is "sweep a spill".
     *
     * A tip needs several balls and we hold at most 4, so every trip should be a full load. If decide() wants to shoot 1-3 balls that
     * cannot finish the tip, collect more first (a group the camera saw, or a tipped Cell's spill), or park if there is nothing to collect.
     */
    private Move chooseMove(Point robot, double secondsLeft) {
        seenCluster = chooseLoad(robot);
        int looseBallsSeen = (seenCluster == null) ? 0 : seenCluster.size();
        boolean atShootPoint = Point.distance(robot, launchPointOf(upCell)) < AT_SHOOT_POINT_MM;
        int ballsInUpCell = ballsIn(upCell);
        int ballsNeeded = RedTippingAuto.ballsNeededToTip(tipsDone);

        // The two "does the trip fit" times are 0: decide() never turns a shot or a trip down for time.
        // Only its rule 1 is left (time left <= "drive to park" + margin -> park). We give it a "drive to park" that makes it
        // park at PARK_WHEN_SECONDS_LEFT.
        double secondsToParkForDecide = PARK_WHEN_SECONDS_LEFT - AutoDecider.PARK_MARGIN_SECONDS;
        Move move = AutoDecider.decide(PARTNER_MODE, secondsLeft, secondsToParkForDecide, 0, 0, ballsHeld, atShootPoint, upCell,
                SharedData.isHiveTipping(), ballsInUpCell, tipsDone, pilesLeft, looseBallsSeen);
        move = limitMove(move);

        // The count is only a hint (where a ball lands changes the weight), so when we already reach the tip count, shoot now.
        boolean shotWillTip = ballsInUpCell + ballsHeld >= ballsNeeded;
        boolean loadTooSmall = isShoot(move) && ballsHeld > 0 && ballsHeld < MOST_BALLS_WE_CAN_HOLD && !shotWillTip;

        Cell spillToSweep = null;
        if (loadTooSmall) {
            if (seenCluster != null) {
                move = Move.GET_SEEN_BALLS;
            } else {
                spillToSweep = newestPendingSpill();
            }
        }

        // decide() says PARK when nothing is left to do. If there is time AND one more full load could still finish a tip,
        // sweep the spill instead. Standing still scores nothing, so with no spill we park now.
        boolean couldStillTip = ballsInUpCell + ballsHeld + MOST_BALLS_WE_CAN_HOLD >= ballsNeeded;
        boolean hasTimeToSweep = secondsLeft > PARK_WHEN_SECONDS_LEFT;
        if (move == Move.PARK && couldStillTip && hasTimeToSweep && ballsHeld == 0) {
            spillToSweep = newestPendingSpill();
        }

        boolean tooLateToSweep = !SWEEP_ENABLED || sweepGivenUp || secondsLeft < SWEEP_MIN_SECONDS_LEFT;
        if (tooLateToSweep) {
            spillToSweep = null;
        }

        KLog.d(TAG, () -> "secondsLeft=" + round(secondsLeft)
                + " ballsHeld=" + ballsHeld + " atShootPoint=" + atShootPoint + " upCell=" + upCell
                + " ballsInUpCell=" + ballsInUpCell + " tipsDone=" + tipsDone + " pilesLeft=" + pilesLeft
                + " spills=" + pendingSpills + " seen=" + looseBallsSeen);

        // From the third tip on (even after it is done), with no balls known to collect: stand where the camera sees the whole up end and keep looking.
        // We never park before PARK_WHEN_SECONDS_LEFT, so when decide() has nothing to do we go there (or stay there) and look again.
        goingToViewpoint = false;
        sweepRunningFor = null;
        boolean nothingToDo = (move == Move.PARK);
        boolean notLookedYetThisTip = viewedAtTipsDone != tipsDone;
        boolean loadTooSmallToShoot = (loadTooSmall || spillToSweep != null) && notLookedYetThisTip; // look once before shooting a small load
        boolean wantToLook = tipsDone >= 2 && seenCluster == null && ballsHeld < MOST_BALLS_WE_CAN_HOLD
                && secondsLeft > PARK_WHEN_SECONDS_LEFT && (nothingToDo || loadTooSmallToShoot);
        if (wantToLook) {
            boolean atViewpoint = Point.distance(robot, viewpointOfUpEnd()) < VIEWPOINT_REACHED_MM;
            boolean hasTimeToDrive = secondsLeft > driveSeconds(robot, viewpointOfUpEnd()) + LOOK_SETTLE_SECONDS + PARK_WHEN_SECONDS_LEFT;
            if (!atViewpoint && hasTimeToDrive) {
                goingToViewpoint = true;
                return Move.WAIT;
            }
            if (nothingToDo) {
                return Move.WAIT; // already there (or no time to drive there): stay and keep looking until it is time to park
            }
        }

        if (spillToSweep != null) {
            sweepRunningFor = spillToSweep;
            return Move.WAIT;
        }
        return move;
    }

    /** Builds the action for the chosen move and remembers what is running. */
    private void startMove(Move move) {
        currentMove = move;
        currentPile = pileOf(move);

        if (goingToViewpoint) {
            viewedAtTipsDone = tipsDone;
            currentAction = driveTo(viewpointOfUpEnd(), lookTowardOfUpEnd(), "viewpoint"); // currentMove is WAIT, but it is really a drive to the viewpoint
        } else if (sweepRunningFor != null) {
            currentAction = sweepMove(sweepRunningFor); // currentMove is WAIT, but it is really a sweep
        } else if (currentPile != null) {
            Point flower = currentPile == Pile.FLOWER_A ? FieldConfig.aFlowerPoint
                    : currentPile == Pile.FLOWER_B ? FieldConfig.bFlowerPoint : null;
            currentAction = pickupMove(pilePointOf(currentPile), "pickup" + currentPile, flower);
        } else if (move == Move.SHOOT_A) {
            currentAction = shootMove(Cell.A);
        } else if (move == Move.SHOOT_B) {
            currentAction = shootMove(Cell.B);
        } else if (move == Move.WAIT) {
            currentAction = new WaitAction(WAIT_MS);
        } else if (move == Move.GET_SEEN_BALLS && seenCluster != null) {
            currentSeenCenter = aimPointOf(seenCluster);
            currentSeenSize = seenCluster.size();
            committedCenter = currentSeenCenter;
            committedBalls = new ArrayList<>(seenCluster.getBalls());
            committedAtSeconds = getRuntime();
            currentAction = pickupMove(currentSeenCenter, "pickupSeen", null);
        } else {
            currentMove = Move.PARK; // PARK, or GET_SEEN_BALLS with nothing seen (decide() cannot do that, but park to be safe)
            stowIntake("park");
            currentAction = driveTo(parkPoint(), null, "park");
        }
    }

    private static Pile pileOf(Move move) {
        if (move == null) {
            return null; // nothing has started yet
        }
        switch (move) {
            case GET_GARDEN:
                return Pile.GARDEN;
            case GET_FLOWER_A:
                return Pile.FLOWER_A;
            case GET_FLOWER_B:
                return Pile.FLOWER_B;
            default:
                return null;
        }
    }

    private static boolean isShoot(Move move) {
        return move == Move.SHOOT_A || move == Move.SHOOT_B;
    }

    private boolean isPickup(Move move) {
        return pileOf(move) != null || move == Move.GET_SEEN_BALLS;
    }

    /** The spill that tipped most recently, or null if there is none. Older spills have mostly rolled away or across the midline. */
    private Cell newestPendingSpill() {
        for (int i = pendingSpills.size() - 1; i >= 0; i--) {
            Cell spill = pendingSpills.get(i);
            boolean spillIsOnHighYEnd = (spill == Cell.A); // A tipped = its balls are on the +y wall, B tipped = the -y wall
            if (!staysOnUpSide() || spillIsOnHighYEnd == upCellIsOnHighYEnd()) {
                return spill;
            }
        }
        return null;
    }

    // =============================================================================================
    // After a move: update what we know
    // =============================================================================================

    /** Called once when the running move is done. */
    private void finishMove() {
        if (goingToViewpoint) {
            goingToViewpoint = false;
            decideNotBeforeSeconds = getRuntime() + LOOK_SETTLE_SECONDS; // we are at the viewpoint: let the camera look, then decide
        }
        if (sweepRunningFor != null) {
            stopSweepIntake();
            pendingSpills.remove(sweepRunningFor); // the whole loop is driven: never go back for the same spill
            sweepCell = null;
            sweepLeg = 0;
            sweepRunningFor = null;
        }
        switch (currentMove) {
            case SHOOT_A:
            case SHOOT_B:
                afterShot();
                break;
            case GET_GARDEN:
            case GET_FLOWER_A:
            case GET_FLOWER_B:
                afterPileTrip();
                break;
            case GET_SEEN_BALLS:
                decideNotBeforeSeconds = getRuntime() + COUNT_SETTLE_SECONDS;
                afterSeenBallsTrip();
                break;
            case PARK:
                parked = true;
                break;
            default:
                break;
        }
        KLog.d(TAG, () -> "finished " + currentMove + ": ballsHeld=" + ballsHeld + " tipsDone=" + tipsDone
                + " upCell=" + upCell + " pilesLeft=" + pilesLeft);
        currentAction = null;
        currentAim = null;
    }

    private void afterShot() {
        Cell shotAt = (currentMove == Move.SHOOT_A) ? Cell.A : Cell.B;
        int ballsNowInCell = ballsIn(shotAt) + ballsHeld;
        ballsHeld = 0;
        if (ballsNowInCell >= RedTippingAuto.ballsNeededToTip(tipsDone)) {
            // The Cell tips down and the other Cell stands up.
            ballsNowInCell = 0;
            tipsDone++;
            upCell = (shotAt == Cell.A) ? Cell.B : Cell.A;
            hiveSettledAtSeconds = getRuntime() + HIVE_TIP_SECONDS; // the Hive swings for a few seconds: do not shoot until it is still
            if (ownsSpillOf(shotAt)) {
                pendingSpills.add(shotAt); // the Cell that tipped down dumped its balls on its own side of the field
            }
        }
        setBallsIn(shotAt, ballsNowInCell);
    }

    private void afterPileTrip() {
        pilesLeft.remove(currentPile); // whether or not the pickup worked, this pile is visited: never go back
        if (pickupFailed(currentPile.toString())) {
            return;
        }
        // The beam-break count lags just after a pickup, and a pile is always full, so we trust our own count.
        ballsHeld = Math.min(MOST_BALLS_WE_CAN_HOLD, ballsHeld + BALLS_PER_PILE);
    }

    private void afterSeenBallsTrip() {
        committedCenter = null;
        committedBalls = new ArrayList<>();
        visitedPlaces.add(currentSeenCenter); // whether or not it worked: a group that is not there any more must not loop
        if (pickupFailed("seen balls")) {
            return;
        }
        // the camera also sees the balls of a pile: if the pickup was at a pile, that pile is taken
        for (Pile pile : new ArrayList<>(pilesLeft)) {
            if (Point.distance(currentSeenCenter, pilePointOf(pile)) < VISITED_REPEAT_MM) {
                pilesLeft.remove(pile);
            }
        }
        int ourCount = Math.min(MOST_BALLS_WE_CAN_HOLD, ballsHeld + currentSeenSize);
        ballsHeld = beamBreakIsTrustworthy() ? SharedData.getBeamBreakData().getCount() : ourCount;
    }

    /** True (and reported) when the pickup that just finished could not plan a route. */
    private boolean pickupFailed(String what) {
        AutoToBallsPathingAction pickup = (AutoToBallsPathingAction) currentAction;
        if (!pickup.getFailed()) {
            return false;
        }
        String why = (pickup.getPlan() == null) ? "no plan" : pickup.getPlan().why;
        KLog.d(TAG, () -> "pickup of " + what + " failed: " + why);
        telemetry.addData("pickup failed", what + ": " + why);
        telemetry.update();
        return true;
    }

    private int ballsIn(Cell cell) {
        return (cell == Cell.A) ? ballsInCellA : ballsInCellB;
    }

    private void setBallsIn(Cell cell, int balls) {
        if (cell == Cell.A) {
            ballsInCellA = balls;
        } else {
            ballsInCellB = balls;
        }
    }

    // =============================================================================================
    // Remembering the balls the camera saw
    // =============================================================================================

    /**
     * Keeps every ball the camera sees for a few seconds, so a ball seen while driving is still known after the robot has turned away.
     * A ball is forgotten when it is old, or when we drove over its spot.
     */
    private void rememberSightings() {
        long now = System.currentTimeMillis();
        long listMs = SharedData.getBallInformationTimeMs();
        if (listMs != 0 && listMs != lastSightingListMs) {
            lastSightingListMs = listMs;
            for (BallInformation seen : SharedData.getBallInformation()) {
                int sameBall = indexOfRememberedBallNear(seen.toPoint());
                if (sameBall >= 0) {
                    rememberedBalls.set(sameBall, seen);
                    rememberedAtMs.set(sameBall, now);
                } else {
                    rememberedBalls.add(seen);
                    rememberedAtMs.add(now);
                }
            }
        }
        Point robot = robotPoint();
        for (int i = rememberedBalls.size() - 1; i >= 0; i--) {
            boolean tooOld = now - rememberedAtMs.get(i) > BALL_MEMORY_MS;
            boolean weDroveOverIt = Point.distance(rememberedBalls.get(i).toPoint(), robot) < BALL_FORGET_NEAR_ROBOT_MM;
            if (tooOld || weDroveOverIt) {
                rememberedBalls.remove(i);
                rememberedAtMs.remove(i);
            }
        }
    }

    private int indexOfRememberedBallNear(Point point) {
        for (int i = 0; i < rememberedBalls.size(); i++) {
            if (Point.distance(rememberedBalls.get(i).toPoint(), point) < BALL_MERGE_MM) {
                return i;
            }
        }
        return -1;
    }

    private boolean nearAVisitedPlace(Point point) {
        for (Point visited : visitedPlaces) {
            if (Point.distance(point, visited) < VISITED_REPEAT_MM) {
                return true;
            }
        }
        return false;
    }

    // =============================================================================================
    // Choosing which balls to go for
    // =============================================================================================

    /**
     * Picks the first stop of the best load. A load is one group of remembered balls, or two groups when the first does not fill the intake.
     * The chosen first group is kept until it is picked up, reached or expired, and replaced only by a load that scores
     * COMMIT_SWITCH_SCORE times better. Returns null when there is nothing worth going for.
     */
    private BallCluster chooseLoad(Point robot) {
        int room = MOST_BALLS_WE_CAN_HOLD - ballsHeld;
        dropCommitmentIfDone(robot);
        List<BallCluster> groups = groupBalls(usableBalls());
        if (groups.isEmpty() || room <= 0) {
            committedCenter = null;
            return null;
        }

        // score[i] = the best load that STARTS with group i
        double[] score = new double[groups.size()];
        int best = 0;
        for (int i = 0; i < groups.size(); i++) {
            score[i] = bestScoreStartingWith(robot, groups, i, room);
            if (score[i] > score[best]) {
                best = i;
            }
        }

        // Keep the group we already chose, unless the best one is clearly better.
        int chosen = best;
        int committed = indexOfGroupAtCommittedCenter(groups);
        if (committed >= 0 && score[best] < COMMIT_SWITCH_SCORE * score[committed]) {
            chosen = committed;
        }
        return groups.get(chosen);
    }

    private void dropCommitmentIfDone(Point robot) {
        if (committedCenter == null) {
            return;
        }
        boolean tooOld = getRuntime() - committedAtSeconds > COMMIT_MAX_SECONDS;
        boolean alreadyVisited = nearAVisitedPlace(committedCenter);
        boolean weAreThere = Point.distance(robot, committedCenter) < COMMIT_REACHED_MM;
        if (tooOld || alreadyVisited || weAreThere) {
            committedCenter = null;
        }
    }

    /** The remembered balls that are ours and not near a place we already went to, plus the balls of the chosen group even if the memory just dropped them. */
    private List<BallInformation> usableBalls() {
        List<BallInformation> usable = new ArrayList<>();
        for (BallInformation ball : rememberedBalls) {
            if (wantsBall(ball) && !nearAVisitedPlace(ball.toPoint())) {
                usable.add(ball);
            }
        }
        if (committedCenter != null) {
            for (BallInformation old : committedBalls) {
                boolean alreadyInList = false;
                for (BallInformation ball : usable) {
                    if (Point.distance(ball.toPoint(), old.toPoint()) < BALL_MERGE_MM) {
                        alreadyInList = true;
                        break;
                    }
                }
                if (!alreadyInList && !nearAVisitedPlace(old.toPoint())) {
                    usable.add(old);
                }
            }
        }
        return usable;
    }

    private int indexOfGroupAtCommittedCenter(List<BallCluster> groups) {
        if (committedCenter == null) {
            return -1;
        }
        for (int i = 0; i < groups.size(); i++) {
            if (Point.distance(aimPointOf(groups.get(i)), committedCenter) < VISITED_REPEAT_MM) {
                return i;
            }
        }
        return -1;
    }

    /** The best score of a load that starts with groups[first]: alone, or followed by one other group when the first does not fill the intake. */
    private double bestScoreStartingWith(Point robot, List<BallCluster> groups, int first, int room) {
        double best = loadScore(robot, groups.get(first), null, room);
        if (groups.get(first).size() < room) {
            for (int second = 0; second < groups.size(); second++) {
                if (second != first) {
                    best = Math.max(best, loadScore(robot, groups.get(first), groups.get(second), room));
                }
            }
        }
        return best;
    }

    /**
     * Tip progress per second of a load: first group, then optionally a second, then the up Cell's launch point, then the shot.
     * A load that finishes the tip is worth 1, otherwise a fraction of its progress.
     */
    private double loadScore(Point robot, BallCluster first, BallCluster second, int room) {
        int balls = Math.min(first.size() + (second == null ? 0 : second.size()), room);
        Point firstPoint = aimPointOf(first);
        Point lastPoint = (second == null) ? firstPoint : aimPointOf(second);

        double seconds = driveSeconds(robot, firstPoint) + LOAD_STOP_SECONDS;
        if (second != null) {
            seconds += driveSeconds(firstPoint, lastPoint) + LOAD_STOP_SECONDS;
        }
        seconds += driveSeconds(lastPoint, launchPointOf(upCell)) + SECONDS_TO_SHOOT;

        int needed = RedTippingAuto.ballsNeededToTip(tipsDone);
        int total = ballsIn(upCell) + ballsHeld + balls;
        double value = (total >= needed) ? 1.0 : LOAD_PARTIAL_VALUE * total / needed;
        return value / seconds;
    }

    /** Remembered balls chained into groups: a ball closer than LOAD_GROUP_MM to a ball of the group joins it. Groups with opponent balls are dropped. */
    private List<BallCluster> groupBalls(List<BallInformation> balls) {
        List<BallCluster> groups = new ArrayList<>();
        boolean[] used = new boolean[balls.size()];
        for (int start = 0; start < balls.size(); start++) {
            if (used[start]) {
                continue;
            }
            BallCluster group = new BallCluster(BallCluster.tileColumnOf(balls.get(start).x), BallCluster.tileRowOf(balls.get(start).y));
            List<Integer> queue = new ArrayList<>(); // balls of this group whose neighbours we still have to look at
            queue.add(start);
            used[start] = true;
            for (int q = 0; q < queue.size(); q++) {
                BallInformation ball = balls.get(queue.get(q));
                group.addBall(ball);
                for (int other = 0; other < balls.size(); other++) {
                    if (!used[other] && Point.distance(ball.toPoint(), balls.get(other).toPoint()) < LOAD_GROUP_MM) {
                        used[other] = true;
                        queue.add(other);
                    }
                }
            }
            if (!group.isEliminated(allianceColor)) {
                groups.add(group);
            }
        }
        return groups;
    }

    /**
     * Where to drive for a group. The middle of a group is only worth driving to when a ball is actually near it; for scattered balls
     * (a nectar and a pollen 400 mm apart) the middle is empty floor, so aim at the ball closest to the middle instead.
     */
    private static Point aimPointOf(BallCluster cluster) {
        Point center = cluster.getCenter();
        Point nearest = center;
        double nearestDistance = Double.MAX_VALUE;
        for (BallInformation ball : cluster.getBalls()) {
            double distance = Point.distance(center, ball.toPoint());
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = ball.toPoint();
            }
        }
        return (nearestDistance <= BALL_NEAR_CENTER_MM) ? center : nearest;
    }

    // =============================================================================================
    // The moves
    // =============================================================================================

    /**
     * Drive to the launch point of the Cell, warm the flywheel on the way, and shoot when the robot is close enough,
     * the flywheel is ready and the turret is aimed at THIS Cell.
     */
    private KActionSet shootMove(Cell cell) {
        Point launch = launchPointOf(cell);
        Point target = (cell == Cell.A) ? FieldConfig.aTargetPoint : FieldConfig.bTargetPoint;

        // A fresh aim per shot, aimed at THIS Cell.
        TurretAutoAlign aim = new TurretAutoAlign(opModeUtilities, turret, target);
        aim.setToleranceDeg(5);
        currentAim = aim;

        AdaptivePurePursuitAction drive = driveTo(launch, target, "driveToLaunch" + cell); // end with the front (and the camera) facing the Cell

        // The shot distance changes while we drive, so the flywheel follows the CURRENT distance to the Cell (not a fixed launch point).
        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, target); // warms up while we drive
        shooterRun.setName("shooterRun");

        ShooterReady shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");

        TurretReadyAuto turretReady = new TurretReadyAuto(aim);
        turretReady.setName("turretReady");

        // The turret tracks the Cell, so we do not need to stop at the launch point: shoot as soon as we are close enough and the Hive is still.
        Action closeEnough = new Action() {
            @Override
            protected void update() {
                if (Point.distance(robotPoint(), launch) < SHOOT_ZONE_MM && getRuntime() >= hiveSettledAtSeconds) {
                    isDone = true;
                }
            }
        };
        closeEnough.setName("closeEnoughToShoot");

        PushBall pushBall = new PushBall(stopper, intake);
        pushBall.setName("shoot");
        pushBall.setDependentActions(closeEnough, turretReady);

        // The balls are gone: stop driving to the launch point. The next move starts from wherever we are.
        Action endDrive = new Action() {
            @Override
            protected void update() {
                drive.setIsDone(true);
                isDone = true;
            }
        };
        endDrive.setName("endDriveToLaunch");
        endDrive.setDependentActions(pushBall);

        TurretStop turretStop = new TurretStop(aim);
        turretStop.setName("turretStop");
        turretStop.setDependentActions(pushBall);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushBall);

        KActionSet set = new KActionSet();
        set.setName("shoot" + cell);
        set.addAction(drive, shooterRun, shooterReady, turretReady, closeEnough, pushBall, endDrive, turretStop, shooterStop);
        return set;
    }

    /** Plan a route to the balls with the path planner, drive it and run the intake (and the Flower arm for a Flower pile). */
    private Action pickupMove(Point ball, String name, Point flower) {
        AutoToBallsPathingAction pickup = new AutoToBallsPathingAction(driveTrain, stopper, intake, ball, true, flower);
        pickup.setName(name);
        return pickup;
    }

    /**
     * Where the robot center parks: in the corner, but far enough from both walls that the whole body fits at any heading
     * (the circle that holds the body: the longer end and half the width).
     */
    private static Point parkPoint() {
        double reach = Math.hypot(Math.max(RobotFootprint.FRONT_MM, RobotFootprint.BACK_MM), RobotFootprint.WIDTH_MM / 2) + PARK_WALL_GAP_MM;
        return new Point(reach, reach);
    }

    // =============================================================================================
    // The sweep
    // =============================================================================================

    /** The 4 corners of the rectangle loop for a Cell's spill: out lane, across the far end, back lane. */
    private List<Point> sweepCorners(Cell tipped) {
        boolean plusY = (tipped == Cell.A); // A tipped = its balls are on the +y wall, B tipped = the -y wall
        double farY = plusY ? PathPlanning.FIELD_SIZE_MM - SWEEP_FAR_WALL_GAP_MM : SWEEP_FAR_WALL_GAP_MM;
        double launchY = (plusY ? launchPointOf(Cell.B) : launchPointOf(Cell.A)).getY(); // the up Cell is the other one
        double nearY = plusY ? launchY + SWEEP_NEAR_INSET_MM : launchY - SWEEP_NEAR_INSET_MM;
        if (staysOnUpSide()) {
            // the loop's near end must not cross the middle of the Hive: keep the whole loop on the up Cell's end
            nearY = plusY ? Math.max(nearY, FieldConfig.MIDLINE_X) : Math.min(nearY, FieldConfig.MIDLINE_X);
        }

        List<Point> corners = new ArrayList<>();
        corners.add(new Point(SWEEP_OUT_X_MM, nearY));
        corners.add(new Point(SWEEP_OUT_X_MM, farY));
        corners.add(new Point(SWEEP_BACK_X_MM, farY));
        corners.add(new Point(SWEEP_BACK_X_MM, nearY));
        return corners;
    }

    /** The rest of the loop as ONE path through all the corners left, so the robot does not stop at each corner. */
    private Action sweepMove(Cell tipped) {
        List<Point> corners = sweepCorners(tipped);
        if (sweepCell != tipped) {
            // A new loop starts at the corner nearest to us: from the far end, drive only the way back, not up to the near end and down again.
            sweepCell = tipped;
            sweepLeg = 0;
            double nearest = Double.MAX_VALUE;
            for (int i = 0; i < corners.size(); i++) {
                double distance = Point.distance(robotPoint(), corners.get(i));
                if (distance < nearest) {
                    nearest = distance;
                    sweepLeg = i;
                }
            }
        }
        sweepIntake = new DriveDirectionIntakeController(intake);
        return driveThrough(corners.subList(sweepLeg, corners.size()), null, "sweep" + tipped, 12000);
    }

    /** While a sweep leg drives: stop when the intake is full, or when a group lies close beside the lane. */
    private void checkSweep() {
        if (sweepRunningFor == null || currentAction == null) {
            return;
        }
        if (sweepIntake != null) {
            sweepIntake.update();
        }
        List<Point> corners = sweepCorners(sweepCell);
        while (sweepLeg < corners.size() - 1 && Point.distance(robotPoint(), corners.get(sweepLeg)) < SWEEP_CORNER_REACHED_MM) {
            sweepLeg++; // passed this corner: a broken-off sweep restarts from the next one
        }
        if (ballsHeld >= MOST_BALLS_WE_CAN_HOLD) {
            dropRunningMove("sweep: holding " + ballsHeld + ", go shoot");
            return;
        }
        long now = System.currentTimeMillis();
        if (now - lastSweepCheckMs < CHECK_EVERY_MS) {
            return;
        }
        lastSweepCheckMs = now;
        Point robot = robotPoint();
        BallCluster cluster = chooseLoad(robot);
        if (cluster != null && distanceToSegment(aimPointOf(cluster), robot, corners.get(sweepLeg)) <= SWEEP_DETOUR_MM) {
            dropRunningMove("sweep: a group of " + cluster.size() + " lies beside the lane"); // the sweep goes on from the same corner later
        }
    }

    private void stopSweepIntake() {
        if (sweepIntake != null) {
            sweepIntake = null;
            stowIntake("sweepDone");
        }
    }

    /** Intake off, Flower arm and both sides up: the robot must end inside its starting size. */
    private void stowIntake(String reason) {
        intake.stop();
        intake.getFlower().raise();
        intake.getSide(BeamBreakPosition.IntakeSide.A).raise(); // a raise waiting on the flower finishes in intake.update()
        intake.getSide(BeamBreakPosition.IntakeSide.B).raise();
        KLog.d(TAG, () -> "event=stowIntake reason=" + reason);
    }

    /** Distance from a point to the straight segment a-b. */
    private static double distanceToSegment(Point p, Point a, Point b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        double lengthSquared = dx * dx + dy * dy;
        double t = (lengthSquared < 1e-9) ? 0
                : Math.max(0, Math.min(1, ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / lengthSquared));
        return Point.distance(p, new Point(a.getX() + t * dx, a.getY() + t * dy));
    }

    // =============================================================================================
    // Safety checks: run out of time, or get stuck
    // =============================================================================================

    /**
     * decide() only runs between moves, so a long pickup or sweep can run past the time to park. Every 250 ms: if the time left is
     * about the drive to the park spot, drop the running pickup or sweep so the next decide() parks.
     */
    private void parkIfOutOfTime() {
        boolean canBeDropped = currentAction != null && currentMove != Move.PARK && !isShoot(currentMove);
        long now = System.currentTimeMillis();
        if (!canBeDropped || now - lastParkCheckMs < CHECK_EVERY_MS) {
            return;
        }
        lastParkCheckMs = now;
        double secondsLeft = AUTO_SECONDS - getRuntime();
        boolean almostOutOfTime = secondsLeft <= PARK_WHEN_SECONDS_LEFT;
        if (almostOutOfTime) {
            if (sweepRunningFor != null) {
                sweepGivenUp = true; // out of time for sweeping: park, do not start the same loop again
            }
            dropRunningMove("time to park");
        }
    }

    /** A pickup that has not finished after PICKUP_GIVE_UP_SECONDS is stuck (for example against a Flower): give up and decide again. */
    private void giveUpIfPickupStuck() {
        boolean stuck = isPickup(currentMove) && currentAction != null && getRuntime() - moveStartedAtSeconds >= PICKUP_GIVE_UP_SECONDS;
        if (!stuck) {
            return;
        }
        dropRunningMove("pickup stuck for " + PICKUP_GIVE_UP_SECONDS + " s");
        decideNotBeforeSeconds = getRuntime() + COUNT_SETTLE_SECONDS;
        if (currentPile != null) {
            pilesLeft.remove(currentPile); // never go back to a pile that jammed us
        }
        if (currentSeenCenter != null) {
            visitedPlaces.add(currentSeenCenter);
        }
    }

    /** Stops the running pickup or sweep leg right now, so the main loop decides again. */
    private void dropRunningMove(String why) {
        KLog.d(TAG, () -> "event=dropMove move=" + currentMove + " reason=" + why);
        if (currentAction instanceof AutoToBallsPathingAction) {
            ((AutoToBallsPathingAction) currentAction).setIsDone(true);
        }
        sweepIntake = null;
        stowIntake("dropMove");
        driveTrain.setPower(0);
        currentAction = null;
        currentAim = null;
        goingToViewpoint = false;
        sweepRunningFor = null; // a dropped sweep leg is NOT counted as done: the sweep can go on from the same corner
    }

    // =============================================================================================
    // Driving
    // =============================================================================================

    /**
     * Drive to a point along the planner's route (around the Hive legs and flowers, on our side).
     *
     * @param faceToward end with the front facing this point. null = keep the way we travel.
     */
    private AdaptivePurePursuitAction driveTo(Point goal, Point faceToward, String name) {
        return driveThrough(Collections.singletonList(goal), faceToward, name, 8000);
    }

    /** One path that goes through every goal in order, each one reached by the planner's route from the one before. */
    private AdaptivePurePursuitAction driveThrough(List<Point> goals, Point faceToward, String name, long timeoutMs) {
        // 1. The planner's route to every goal, joined into one list of points.
        List<Point> points = new ArrayList<>();
        Point previous = robotPoint();
        for (Point goal : goals) {
            List<Point> route = planner.plan(previous, goal);
            if (route == null || route.isEmpty()) {
                KLog.d(TAG, () -> "no planned route to " + goal + ", driving straight");
                route = Collections.singletonList(goal);
            }
            points.addAll(route);
            previous = goal;
        }

        // 2. Drive it. At every point the robot faces the way it travels.
        AdaptivePurePursuitAction drive = new AdaptivePurePursuitAction(driveTrain);
        drive.setName(name);
        drive.setPlanStartFrom(null); // plan from where the robot is NOW
        drive.setFinalAngleLockingThresholdDeg(30);
        drive.setLookAheadRadius(125);
        drive.setFinalSearchRadiusMM(150);
        drive.setMaxTimeOutMS(timeoutMs);

        Point from = robotPoint();
        double heading = Math.toDegrees(new Position(SharedData.getOdometryWheelIMUPosition()).getTheta());
        for (int i = 0; i < points.size(); i++) {
            Point p = points.get(i);
            boolean last = (i == points.size() - 1);
            if (Point.distance(from, p) > 1e-6) {
                heading = Math.toDegrees(Math.atan2(p.getY() - from.getY(), p.getX() - from.getX()));
            }
            double headingHere = (last && faceToward != null) ? frontToward(p, faceToward) : heading;
            drive.addPoint(p.getX(), p.getY(), headingHere);
            from = p;
        }
        return drive;
    }

    /**
     * A new drive plans its path one small step per loop, and the robot stands still until that is done. Do all the
     * steps right now instead, so the robot starts moving at once.
     */
    private void precomputePathsNow() {
        int steps = 0;
        while (!AdaptivePurePursuitAction.allPrecomputeDone() && steps++ < 10000) {
            AdaptivePurePursuitAction.runPrecomputeStepForAll();
        }
    }

    /**
     * The heading that points the FRONT of the robot at the point. The camera is on the front, so facing the Hive while shooting gives
     * it the best view of the field (and the turret then sits near 0 degrees, nowhere near its +-180 wrap).
     */
    private static double frontToward(Point from, Point toward) {
        double deg = MathFunctions.angleWrapDeg(Math.toDegrees(Math.atan2(toward.getY() - from.getY(), toward.getX() - from.getX())));
        return deg;
    }

    // =============================================================================================
    // Small helpers
    // =============================================================================================

    Point robotPoint() {
        return new Position(SharedData.getOdometryWheelIMUPosition()).toPoint();
    }

    static Point launchPointOf(Cell cell) {
        return (cell == Cell.A) ? FieldConfig.aLaunchPoint : FieldConfig.bLaunchPoint;
    }

    private static Point pilePointOf(Pile pile) {
        switch (pile) {
            case GARDEN:   // the middle of its 4 balls, which lie along the wall
                return new Point(GARDEN_BALLS_CENTER_X_MM, FieldConfig.gardenPoint.getY());
            case FLOWER_A: // the open side of Flower A is +x (away from the wall)
                return FieldConfig.flowerPilePoint(FieldConfig.aFlowerPoint);
            default:       // the open side of Flower B is +y (away from the wall)
                return FieldConfig.flowerPilePoint(FieldConfig.bFlowerPoint);
        }
    }

    /** Seconds to drive from one point to another along the planner's route (a straight line if there is none). */
    double driveSeconds(Point from, Point to) {
        List<Point> route = planner.plan(from, to);
        double length = 0;
        Point previous = from;
        if (route == null || route.isEmpty()) {
            length = Point.distance(from, to);
        } else {
            for (Point p : route) {
                length += Point.distance(previous, p);
                previous = p;
            }
        }
        return Math.max(length, Point.distance(from, to)) / SPEED_MM_PER_SECOND; // never less than the straight line
    }

    private static String round(double number) {
        return String.format("%.1f", number);
    }
}
