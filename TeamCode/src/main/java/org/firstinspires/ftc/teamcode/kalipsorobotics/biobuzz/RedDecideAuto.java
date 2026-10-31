package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.beambreak.BeamBreakCountAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.AutoToBallsPathingAction;
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
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakData;
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
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

/**
 * The AUTO where decide() really drives the robot (the 10-18 plan, stage 2).
 *
 * How it works:
 *   1. We start holding the 4 preload balls, so the first answer is "shoot the up Cell".
 *   2. Every time the current move is DONE, we update what we know (balls held, balls in the Cell,
 *      tips, piles left) and ask AutoDecider.decide() for the NEXT move.
 *   3. Driving goes through the path planner (PathPlanning): it routes around the Hive legs and the
 *      flowers and keeps us on our own side. Pile pickups use AutoToBallsPathingAction.
 *
 * Shooting: the flywheel warms up while the robot drives. The ball is pushed only when the robot has
 * arrived AND the flywheel is ready AND the turret is aimed at the Cell we shoot (a fresh
 * TurretAutoAlign per shot, because a TurretAutoAlign never changes the point it aims at).
 *
 * Every number marked GUESS is unmeasured. The balls we hold are COUNTED by this class (a pile gives
 * 4, a shot empties the robot); reading the beam breaks is a later step.
 */
@Autonomous(name = "Red Decide Auto", group = "biobuzz")
public class RedDecideAuto extends KOpMode {

    // ----- Settings -----------------------------------------------------------------------------
    private static final double AUTO_SECONDS = 30;                 // manual 10.4
    private static final PartnerMode PARTNER_MODE = PartnerMode.SOLO_NOTHING;
    private static final int PRELOAD_BALLS = 4;                    // manual G304
    private static final int BALLS_PER_PILE = 4;                   // manual 10.3.1
    private static final int MOST_BALLS_WE_CAN_HOLD = 4;           // manual G407
    private static final int NECTAR_IN_CELL_A_AT_START = 3;        // manual 10.3.1

    private static final double SPEED_MM_PER_SECOND = 1000;        // GUESS: how fast the robot drives
    private static final double SECONDS_TO_PICK_UP = 2;            // GUESS
    private static final double SECONDS_TO_SHOOT = 2;              // GUESS
    private static final double WAIT_MS = 500;                     // how long one WAIT lasts

    private static final double FLOWER_BALL_OFFSET_MM = 150;       // GUESS: the pickup target is this far from the Flower's center
                                                                   // (the center itself is inside the Flower, so the planner refuses it)
    private static final double GARDEN_BALLS_CENTER_X_MM = 142;    // GUESS from the sim field: the 4 Garden balls sit at x = 36 to 249
    private static final double OBSTACLE_BUFFER_MM = 350;          // same as DecideSimulation / AutoToBallsPathingAction
    private static final double AT_SHOOT_POINT_MM = 150;           // closer than this to the launch point = "at the shoot point"
    private static final double PARK_WALL_GAP_MM = 15;             // GUESS: room left between the body and the walls at the park spot
    private static final double WATCH_MS = 250;                    // nothing in view and time left: stand still this long, then look again
    // Where to stand to look at the balls a tipped Cell dumped. A tipped = its balls are on the +y wall side, B tipped = the -y wall side.
    // Both on our half, about 1150-1200 mm from the wall, facing the wall (the view is about 2.4 m wide there). GUESS: tune from the sim.
    private static final int BIG_CLUSTER_BALLS = 3;   // a seen cluster this big is taken before the next pile
    private static final Point VIEW_PLUS_Y = new Point(700, 2500);
    private static final double VIEW_PLUS_Y_HEADING_DEG = 90;
    private static final Point VIEW_MINUS_Y = new Point(700, 1100);
    private static final double VIEW_MINUS_Y_HEADING_DEG = -90;
    private static final double LOOK_SETTLE_MS = 1500;             // after arriving, let the rolling balls settle and the camera look
    private static final double COUNT_SETTLE_SECONDS = 0.4;        // after a pickup the beam-break count lags a moment: wait before deciding
    private static final double BALL_NEAR_CENTER_MM = 100;        // a cluster's middle counts as "on a ball" when one is this close
    private static final long SIGHTING_MAX_AGE_MS = 500;           // a camera list older than this is ignored
    private static final double SEEN_CLUSTER_REPEAT_MM = 300;      // balls this close to a cluster we already visited are ignored

    // ----- What we know -------------------------------------------------------------------------
    private int ballsHeld = PRELOAD_BALLS;
    private Cell upCell = Cell.A;
    private int ballsInCellA = NECTAR_IN_CELL_A_AT_START;
    private int ballsInCellB = 0;
    private int tipsDone = 0;
    private final Set<Pile> pilesLeft = EnumSet.allOf(Pile.class);

    // ----- The move that is running -------------------------------------------------------------
    private Move currentMove = null;
    private Action currentAction = null;
    private Pile currentPile = null;                    // for GET_* moves
    private TurretAutoAlign currentAim = null;          // for SHOOT_* moves; updated every loop
    private boolean parked = false;
    private double decideNotBeforeSeconds = 0;          // do not ask decide() before this run time (the ball count is still settling)
    private Point currentSeenCenter = null;             // where the running GET_SEEN_BALLS move is going
    private int currentSeenSize = 0;
    private BallCluster seenCluster = null;             // the cluster the camera sees right now that we would go for
    private final List<Cell> pendingSpills = new ArrayList<>();   // Cells that tipped and whose spilled balls we have not looked at yet
    private Cell lookingAt = null;                      // the spill the running LOOK move is going to
    private final List<Point> visitedSeenClusters = new ArrayList<>(); // where we already went for seen balls (success or not)

    // ----- Robot --------------------------------------------------------------------------------
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Turret turret;
    private BeamBreakCountAction beamBreaks;
    private PathPlanning planner;

    @Override
    protected void initializeRobotConfig() {
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
        TurretConfig.TICKS_INIT_OFFSET = 0;
        TurretConfig.X_INIT_SETUP_MM = FieldConfig.aTargetPoint.getX(); // the turret starts aimed at the first Cell
        TurretConfig.Y_INIT_SETUP_MM = FieldConfig.aTargetPoint.getY();
    }

    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);

        sleep(1000); // let hardware initialize

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 171.5 + 1218, 3658 - 171.5, Math.toRadians(-90));
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        intake = new Intake(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);

        beamBreaks = new BeamBreakCountAction(opModeUtilities); // counts the balls inside the robot

        // AUTO: the opponent's half is blocked. Nothing is tipping yet, so the space under the Hive is open.
        planner = PathPlanning.forFieldAuto(OBSTACLE_BUFFER_MM, RobotFootprint.WIDTH_MM / 2, false);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStartPrecomputingPaths();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();

            // Keep the ball count fresh. While we shoot the count falls, so it is not read then: we need the count
            // from just BEFORE the shot to know how many balls went into the Cell.
            beamBreaks.updateCheckDone();
            boolean shooting = (currentMove == Move.SHOOT_A || currentMove == Move.SHOOT_B) && currentAction != null;
            BeamBreakData balls = SharedData.getBeamBreakData();
            if (!shooting && balls.hasData() && balls.isFresh(250) && balls.isReliable()) {
                ballsHeld = balls.getCount();
            }

            // The turret only moves when its action is updated.
            if (currentAim != null) {
                currentAim.updateCheckDone();
            }

            if (currentAction != null && currentAction.updateCheckDone()) {
                finishMove();
            }
            if (currentAction == null && !parked && getRuntime() >= decideNotBeforeSeconds) {
                startNextMove();
            }

            KLog.d("Odometry", () -> "Position: " + SharedData.getOdometryWheelIMUPosition());
        }
        cleanupRobot();
    }

    // =============================================================================================
    // Asking decide() and starting the answer
    // =============================================================================================

    private void startNextMove() {
        Point robot = robotPoint();
        double secondsLeft = AUTO_SECONDS - getRuntime();
        Point upLaunch = launchPointOf(upCell);
        Pile nextPile = AutoDecider.nextPile(PARTNER_MODE, pilesLeft);

        double secondsToPark = driveSeconds(robot, parkPoint());
        double secondsForShotThenPark = driveSeconds(robot, upLaunch) + SECONDS_TO_SHOOT + driveSeconds(upLaunch, parkPoint());
        double secondsForTripThenPark = secondsForShotThenPark;
        seenCluster = bestSeenCluster(robot);
        int looseBallsSeen = (seenCluster == null) ? 0 : seenCluster.size();
        if (nextPile == null && seenCluster != null) {
            Point cluster = aimPointOf(seenCluster);
            secondsForTripThenPark = driveSeconds(robot, cluster) + SECONDS_TO_PICK_UP
                    + driveSeconds(cluster, upLaunch) + SECONDS_TO_SHOOT + driveSeconds(upLaunch, parkPoint());
        }
        if (nextPile != null) {
            Point pile = pilePointOf(nextPile);
            secondsForTripThenPark = driveSeconds(robot, pile) + SECONDS_TO_PICK_UP
                    + driveSeconds(pile, upLaunch) + SECONDS_TO_SHOOT + driveSeconds(upLaunch, parkPoint());
        }
        boolean atShootPoint = Point.distance(robot, upLaunch) < AT_SHOOT_POINT_MM;
        int ballsInUpCell = (upCell == Cell.A) ? ballsInCellA : ballsInCellB;
        boolean hiveIsTipping = SharedData.isHiveTipping();

        Move move = AutoDecider.decide(PARTNER_MODE, secondsLeft, secondsToPark, secondsForShotThenPark,
                secondsForTripThenPark, ballsHeld, atShootPoint, upCell, hiveIsTipping, ballsInUpCell,
                tipsDone, pilesLeft, looseBallsSeen);

        // A tip needs 8 pollen and we hold at most 4, so every trip should be a full 4. If decide() wants to shoot 1-3 balls and
        // there is time to get more (a cluster in view, or a tipped Cell's spill we have not looked at), collect first.
        String changed = null;
        Cell lookCell = null;
        boolean shootingFew = (move == Move.SHOOT_A || move == Move.SHOOT_B) && ballsHeld > 0 && ballsHeld < MOST_BALLS_WE_CAN_HOLD;
        if (shootingFew) {
            if (seenCluster != null && secondsLeft > secondsForClusterTrip(robot, aimPointOf(seenCluster), upLaunch) + AutoDecider.PARK_MARGIN_SECONDS) {
                move = Move.GET_SEEN_BALLS;
                changed = "COLLECT first: holding " + ballsHeld + " of " + MOST_BALLS_WE_CAN_HOLD + ", a cluster is in view";
            } else {
                Cell spill = nearestPendingSpill(robot);
                if (spill != null && secondsLeft > secondsForLookTrip(robot, spill, upLaunch) + AutoDecider.PARK_MARGIN_SECONDS) {
                    lookCell = spill;
                    changed = "COLLECT first: holding " + ballsHeld + " of " + MOST_BALLS_WE_CAN_HOLD + ", go look at the " + spill + " spill";
                }
            }
        }

        // A Cell that just tipped dumped a lot of balls on its own side, and they roll away fast. Empty-handed, look there before going
        // to a pile (the piles stay where they are).
        boolean goingToPile = move == Move.GET_GARDEN || move == Move.GET_FLOWER_A || move == Move.GET_FLOWER_B;
        if (goingToPile && ballsHeld == 0 && seenCluster != null && seenCluster.size() >= BIG_CLUSTER_BALLS
                && secondsLeft > secondsForClusterTrip(robot, aimPointOf(seenCluster), upLaunch) + AutoDecider.PARK_MARGIN_SECONDS) {
            changed = move + " -> GET_SEEN_BALLS (a cluster of " + seenCluster.size() + " is in view, as good as a pile)";
            move = Move.GET_SEEN_BALLS;
            goingToPile = false;
        }
        if (goingToPile && ballsHeld == 0 && lookCell == null) {
            Cell spill = nearestPendingSpill(robot);
            if (spill != null && secondsLeft > secondsForLookTrip(robot, spill, upLaunch) + AutoDecider.PARK_MARGIN_SECONDS) {
                lookCell = spill;
                changed = move + " -> LOOK at the " + spill + " spill first (fresh spill, empty)";
            }
        }

        // decide() says PARK when nothing is left to do, but parking early wastes the match. Park only when it is time (decide() rule 1,
        // the time left is no more than the drive to the park spot plus a margin). Until then look at a tipped Cell's spill if we have not,
        // otherwise stand still and keep watching the camera.
        boolean watchInstead = (move == Move.PARK) && (secondsLeft > secondsToPark + AutoDecider.PARK_MARGIN_SECONDS);
        if (watchInstead && ballsHeld == 0) {
            Cell spill = nearestPendingSpill(robot);
            if (spill != null && secondsLeft > secondsForLookTrip(robot, spill, upLaunch) + AutoDecider.PARK_MARGIN_SECONDS) {
                lookCell = spill;
                changed = "PARK -> LOOK at the " + spill + " spill (time left, empty)";
            }
        }
        // Nothing to look at: do not stand next to the up Cell. The balls are on the side of the Cell that tipped (the down one),
        // so wait at its viewpoint, where the camera sees them.
        if (changed == null && watchInstead && ballsHeld == 0 && tipsDone > 0) {
            Cell down = (upCell == Cell.A) ? Cell.B : Cell.A;
            Point view = viewpointOf(down);
            if (Point.distance(robot, view) > AT_SHOOT_POINT_MM * 2
                    && secondsLeft > driveSeconds(robot, view) + driveSeconds(view, parkPoint()) + AutoDecider.PARK_MARGIN_SECONDS) {
                lookCell = down;
                changed = "PARK -> go wait at the " + down + " (down) side, where the balls are";
            }
        }
        if (changed == null && watchInstead) {
            changed = "PARK -> WATCH (time left, wait for a cluster in view)";
        }

        String inputs = "secondsLeft=" + round(secondsLeft) + " secondsToPark=" + round(secondsToPark)
                + " shotThenPark=" + round(secondsForShotThenPark) + " tripThenPark=" + round(secondsForTripThenPark)
                + " ballsHeld=" + ballsHeld + " atShootPoint=" + atShootPoint + " upCell=" + upCell
                + " ballsInUpCell=" + ballsInUpCell + " tipsDone=" + tipsDone + " pilesLeft=" + pilesLeft
                + " spills=" + pendingSpills
                + " beam=" + SharedData.getBeamBreakData().getCount() + (SharedData.getBeamBreakData().isFresh(250) ? "" : "(stale)")
                + (SharedData.getBeamBreakData().isReliable() ? "" : "(unreliable)")
                + " seen=" + looseBallsSeen + (seenCluster == null ? "" : " at " + aimPointOf(seenCluster));
        final Move finalMove = move;
        final String finalChanged = changed;
        KLog.d("Decide", () -> inputs + " -> " + finalMove + (finalChanged == null ? "" : " (" + finalChanged + ")"));
        telemetry.addData("decide says", changed == null ? move : changed);
        telemetry.addData("inputs", inputs);
        telemetry.update();

        currentMove = move;
        currentPile = null;
        if (lookCell != null) {
            currentMove = Move.WAIT;
            lookingAt = lookCell;
            currentAction = lookMove(lookCell);
            precomputePathsNow();
            return;
        }
        if (watchInstead) {
            currentMove = Move.WAIT;
            currentAction = new WaitAction(WATCH_MS);
            return;
        }
        switch (move) {
            case SHOOT_A:
                currentAction = shootMove(Cell.A);
                break;
            case SHOOT_B:
                currentAction = shootMove(Cell.B);
                break;
            case GET_GARDEN:
                currentPile = Pile.GARDEN;
                currentAction = pickupMove(Pile.GARDEN);
                break;
            case GET_FLOWER_A:
                currentPile = Pile.FLOWER_A;
                currentAction = pickupMove(Pile.FLOWER_A);
                break;
            case GET_FLOWER_B:
                currentPile = Pile.FLOWER_B;
                currentAction = pickupMove(Pile.FLOWER_B);
                break;
            case WAIT:
                currentAction = new WaitAction(WAIT_MS);
                break;
            case GET_SEEN_BALLS:
                if (seenCluster != null) {
                    currentSeenCenter = aimPointOf(seenCluster);
                    currentSeenSize = seenCluster.size();
                    currentAction = pickupMove(currentSeenCenter, "pickupSeen");
                    break;
                }
                // decide() only answers this when it was told about seen balls, so this cannot happen; park to be safe
                currentMove = Move.PARK;
                currentAction = driveTo(parkPoint(), null, null, "park");
                break;
            case PARK:
            default:
                currentMove = Move.PARK;
                currentAction = driveTo(parkPoint(), null, null, "park");
                break;
        }
        precomputePathsNow();
    }

    /** Called once when the running move is done: update what we know. */
    private void finishMove() {
        if (lookingAt != null) {
            pendingSpills.remove(lookingAt); // looked at it once: never go back for the same spill
            lookingAt = null;
        }
        switch (currentMove) {
            case SHOOT_A:
            case SHOOT_B:
                afterShot();
                break;
            case GET_GARDEN:
            case GET_FLOWER_A:
            case GET_FLOWER_B:
                decideNotBeforeSeconds = getRuntime() + COUNT_SETTLE_SECONDS;
                afterPickup();
                break;
            case GET_SEEN_BALLS:
                decideNotBeforeSeconds = getRuntime() + COUNT_SETTLE_SECONDS;
                afterSeenPickup();
                break;
            case PARK:
                parked = true;
                break;
            default:
                break;
        }
        KLog.d("Decide", () -> "finished " + currentMove + ": ballsHeld=" + ballsHeld + " tipsDone=" + tipsDone
                + " upCell=" + upCell + " pilesLeft=" + pilesLeft);
        currentAction = null;
        currentAim = null;
    }

    private void afterShot() {
        Cell shotAt = (currentMove == Move.SHOOT_A) ? Cell.A : Cell.B;
        int balls = (shotAt == Cell.A) ? ballsInCellA : ballsInCellB;
        balls += ballsHeld;
        ballsHeld = 0;
        if (balls >= RedTippingAuto.ballsNeededToTip(tipsDone)) {
            // The Cell tips down and the other Cell stands up.
            balls = 0;
            tipsDone++;
            upCell = (shotAt == Cell.A) ? Cell.B : Cell.A;
            pendingSpills.add(shotAt); // the Cell that tipped down dumped its balls on its own side of the field
        }
        if (shotAt == Cell.A) {
            ballsInCellA = balls;
        } else {
            ballsInCellB = balls;
        }
    }

    private void afterPickup() {
        // Whether or not the pickup worked, this pile is visited: never go back to it.
        pilesLeft.remove(currentPile);
        AutoToBallsPathingAction pickup = (AutoToBallsPathingAction) currentAction;
        if (pickup.getFailed()) {
            String why = (pickup.getPlan() == null) ? "no plan" : pickup.getPlan().why;
            KLog.d("Decide", () -> "pickup of " + currentPile + " failed: " + why);
            telemetry.addData("pickup failed", currentPile + ": " + why);
            telemetry.update();
            return;
        }
        ballsHeld = ballsInRobot(Math.min(MOST_BALLS_WE_CAN_HOLD, ballsHeld + BALLS_PER_PILE));
    }

    /** Whether or not it worked, never go back to this place: a cluster that is not there any more must not loop. */
    private void afterSeenPickup() {
        visitedSeenClusters.add(currentSeenCenter);
        AutoToBallsPathingAction pickup = (AutoToBallsPathingAction) currentAction;
        if (pickup.getFailed()) {
            String why = (pickup.getPlan() == null) ? "no plan" : pickup.getPlan().why;
            KLog.d("Decide", () -> "pickup of seen balls at " + currentSeenCenter + " failed: " + why);
            telemetry.addData("pickup failed", "seen balls: " + why);
            telemetry.update();
            return;
        }
        // the camera also sees the balls of a pile: if the pickup was at a pile, that pile is taken
        for (Pile pile : new ArrayList<>(pilesLeft)) {
            if (Point.distance(currentSeenCenter, pilePointOf(pile)) < SEEN_CLUSTER_REPEAT_MM) {
                pilesLeft.remove(pile);
            }
        }
        ballsHeld = ballsInRobot(Math.min(MOST_BALLS_WE_CAN_HOLD, ballsHeld + currentSeenSize));
    }

    // =============================================================================================
    // Seeing balls (the camera)
    // =============================================================================================

    /**
     * The best cluster of loose balls the camera sees right now, or null. Uses the team's own chooser (ClusterDetectionAction groups
     * the balls into tiles and BallCluster scores them). "If and only if it sees them": nothing is remembered, so no sighting = null.
     */
    private BallCluster bestSeenCluster(Point robot) {
        long age = System.currentTimeMillis() - SharedData.getBallInformationTimeMs();
        if (SharedData.getBallInformationTimeMs() == 0 || age > SIGHTING_MAX_AGE_MS) {
            return null;
        }
        List<BallInformation> usable = new ArrayList<>();
        for (BallInformation ball : SharedData.getBallInformation()) {
            if (!RedTippingAuto.isOnOurSide(ball.x) || nearAVisitedCluster(ball.toPoint())) {
                continue;
            }
            usable.add(ball);
        }
        ClusterDetectionAction chooser = new ClusterDetectionAction(usable, robot, allianceColor);
        int steps = 0;
        while (!chooser.isFinished() && steps++ < 100000) {
            chooser.updateCheckDone();
        }
        return chooser.getBestCluster();
    }

    private boolean nearAVisitedCluster(Point ball) {
        for (Point visited : visitedSeenClusters) {
            if (Point.distance(ball, visited) < SEEN_CLUSTER_REPEAT_MM) {
                return true;
            }
        }
        return false;
    }

    // =============================================================================================
    // The moves
    // =============================================================================================

    /**
     * Drive to the launch point of the Cell, warm the flywheel on the way, and shoot when the robot is there,
     * the flywheel is ready and the turret is aimed at THIS Cell.
     */
    private KActionSet shootMove(Cell cell) {
        Point launch = launchPointOf(cell);
        Point target = (cell == Cell.A) ? FieldConfig.aTargetPoint : FieldConfig.bTargetPoint;

        // TurretAutoAlign reads its target point when it is built, so set the target first.
        TurretConfig.X_INIT_SETUP_MM = target.getX();
        TurretConfig.Y_INIT_SETUP_MM = target.getY();
        TurretAutoAlign aim = new TurretAutoAlign(opModeUtilities, turret, allianceColor);
        aim.setToleranceDeg(5);
        currentAim = aim;

        AdaptivePurePursuitAction drive = driveTo(launch, null, target, "driveToLaunch" + cell); // end with the front (and the camera) facing the Cell

        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, target, launch); // warms up while we drive
        shooterRun.setName("shooterRun");

        ShooterReady shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        shooterReady.setDependentActions(drive);

        TurretReadyAuto turretReady = new TurretReadyAuto(aim);
        turretReady.setName("turretReady");

        PushBall pushBall = new PushBall(stopper, intake);
        pushBall.setName("shoot");
        pushBall.setDependentActions(drive, turretReady);

        TurretStop turretStop = new TurretStop(aim);
        turretStop.setName("turretStop");
        turretStop.setDependentActions(pushBall);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushBall);

        KActionSet set = new KActionSet();
        set.setName("shoot" + cell);
        set.addAction(drive, shooterRun, shooterReady, turretReady, pushBall, turretStop, shooterStop);
        return set;
    }

    /**
     * Where to drive for a cluster. The middle of a cluster is only worth driving to when a ball is actually near it; for scattered balls
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

    /** The balls in the robot: the beam breaks when they can be trusted, otherwise our own count. */
    private int ballsInRobot(int ourCount) {
        BeamBreakData beam = SharedData.getBeamBreakData();
        return (beam.hasData() && beam.isFresh(250) && beam.isReliable()) ? beam.getCount() : ourCount;
    }

    // ----- Looking at a tipped Cell's spill -----

    private static Point viewpointOf(Cell tipped) {
        return (tipped == Cell.A) ? VIEW_PLUS_Y : VIEW_MINUS_Y;
    }

    private static double viewHeadingOf(Cell tipped) {
        return (tipped == Cell.A) ? VIEW_PLUS_Y_HEADING_DEG : VIEW_MINUS_Y_HEADING_DEG;
    }

    /** The spill whose viewpoint is the shortest drive from here, or null if there is none left to look at. */
    private Cell nearestPendingSpill(Point robot) {
        Cell nearest = null;
        double nearestSeconds = Double.MAX_VALUE;
        for (Cell tipped : pendingSpills) {
            double seconds = driveSeconds(robot, viewpointOf(tipped));
            if (seconds < nearestSeconds) {
                nearest = tipped;
                nearestSeconds = seconds;
            }
        }
        return nearest;
    }

    /** Drive to the viewpoint facing the wall, then stand still a moment so the rolling balls settle and the camera looks. */
    private KActionSet lookMove(Cell tipped) {
        AdaptivePurePursuitAction drive = driveTo(viewpointOf(tipped), viewHeadingOf(tipped), null, "driveToView" + tipped);
        WaitAction settle = new WaitAction(LOOK_SETTLE_MS);
        settle.setName("settle");
        settle.setDependentActions(drive);
        KActionSet set = new KActionSet();
        set.setName("look" + tipped);
        set.addAction(drive, settle);
        return set;
    }

    /** Drive to a cluster, pick it up, drive to the up Cell's launch point, shoot, drive to park. */
    private double secondsForClusterTrip(Point robot, Point cluster, Point upLaunch) {
        return driveSeconds(robot, cluster) + SECONDS_TO_PICK_UP + driveSeconds(cluster, upLaunch)
                + SECONDS_TO_SHOOT + driveSeconds(upLaunch, parkPoint());
    }

    /** Drive to the viewpoint, look, pick up (guessing the balls are near the viewpoint), shoot, park. */
    private double secondsForLookTrip(Point robot, Cell tipped, Point upLaunch) {
        Point view = viewpointOf(tipped);
        return driveSeconds(robot, view) + LOOK_SETTLE_MS / 1000.0 + SECONDS_TO_PICK_UP + driveSeconds(view, upLaunch)
                + SECONDS_TO_SHOOT + driveSeconds(upLaunch, parkPoint());
    }

    /**
     * Where the robot center parks: in the corner, but far enough from both walls that the whole body fits at any heading
     * (the circle that holds the body: the longer end and half the width). The old fixed point was inside the wall clearance, so the
     * robot ended wedged against the wall.
     */
    private static Point parkPoint() {
        double reach = Math.hypot(Math.max(RobotFootprint.FRONT_MM, RobotFootprint.BACK_MM), RobotFootprint.WIDTH_MM / 2) + PARK_WALL_GAP_MM;
        return new Point(reach, reach);
    }

    /** Plan a route to the pile with the path planner, drive it and run the intake. */
    private Action pickupMove(Pile pile) {
        return pickupMove(pilePointOf(pile), "pickup" + pile);
    }

    private Action pickupMove(Point ball, String name) {
        AutoToBallsPathingAction pickup = new AutoToBallsPathingAction(driveTrain, stopper, intake, ball, true);
        pickup.setName(name);
        return pickup;
    }

    /** Drive to a point along the planner's route (around the Hive legs and flowers, on our side). */
    /**
     * @param finalHeadingDeg the heading to end with, or null
     * @param faceToward      when finalHeadingDeg is null: end with the front facing this point. null = just keep the way we travel.
     */
    private AdaptivePurePursuitAction driveTo(Point goal, Double finalHeadingDeg, Point faceToward, String name) {
        Point robot = robotPoint();
        List<Point> route = planner.plan(robot, goal);
        if (route == null || route.isEmpty()) {
            KLog.d("Decide", () -> "no planned route from " + robot + " to " + goal + ", driving straight");
            route = new ArrayList<>();
            route.add(goal);
        }

        AdaptivePurePursuitAction move = new AdaptivePurePursuitAction(driveTrain);
        move.setName(name);
        move.setPlanStartFrom(null); // plan from where the robot is NOW
        move.setFinalAngleLockingThresholdDeg(30);
        move.setLookAheadRadius(125);
        move.setFinalSearchRadiusMM(150);
        move.setMaxTimeOutMS(8000);

        Point previous = robot;
        double heading = Math.toDegrees(new Position(SharedData.getOdometryWheelIMUPosition()).getTheta());
        for (int i = 0; i < route.size(); i++) {
            Point p = route.get(i);
            boolean last = (i == route.size() - 1);
            if (Point.distance(previous, p) > 1e-6) {
                heading = Math.toDegrees(Math.atan2(p.getY() - previous.getY(), p.getX() - previous.getX()));
            }
            if (last && finalHeadingDeg != null) {
                heading = finalHeadingDeg;
            } else if (last && faceToward != null) {
                heading = frontToward(p, faceToward);
            }
            move.addPoint(p.getX(), p.getY(), heading);
            previous = p;
        }
        return move;
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
        final int done = steps;
        KLog.d("Decide", () -> "path precompute took " + done + " steps");
    }

    /**
     * The heading that points the FRONT of the robot at the point. The camera is on the front, so facing the Hive while shooting gives
     * it the best view of the field (and the turret then sits near 0 degrees, nowhere near its +-180 wrap).
     */
    private static double frontToward(Point from, Point toward) {
        return normalizeDeg(Math.toDegrees(Math.atan2(toward.getY() - from.getY(), toward.getX() - from.getX())));
    }

    private static double normalizeDeg(double deg) {
        double d = deg % 360;
        if (d > 180) {
            d -= 360;
        } else if (d <= -180) {
            d += 360;
        }
        return d;
    }

    // =============================================================================================
    // Small helpers
    // =============================================================================================

    private Point robotPoint() {
        return new Position(SharedData.getOdometryWheelIMUPosition()).toPoint();
    }

    private static Point launchPointOf(Cell cell) {
        return (cell == Cell.A) ? FieldConfig.aLaunchPoint : FieldConfig.bLaunchPoint;
    }

    private static Point pilePointOf(Pile pile) {
        switch (pile) {
            case GARDEN:   // the middle of its 4 balls, which lie along the wall
                return new Point(GARDEN_BALLS_CENTER_X_MM, FieldConfig.gardenPoint.getY());
            case FLOWER_A: // the open side of Flower A is +x (away from the wall)
                return new Point(FieldConfig.aFlowerPoint.getX() + FLOWER_BALL_OFFSET_MM, FieldConfig.aFlowerPoint.getY());
            default:       // the open side of Flower B is +y (away from the wall)
                return new Point(FieldConfig.bFlowerPoint.getX(), FieldConfig.bFlowerPoint.getY() + FLOWER_BALL_OFFSET_MM);
        }
    }

    /** Seconds to drive from one point to another along the planner's route (a straight line if there is none). */
    private double driveSeconds(Point from, Point to) {
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
        return length / SPEED_MM_PER_SECOND;
    }

    private static String round(double number) {
        return String.format("%.1f", number);
    }
}
