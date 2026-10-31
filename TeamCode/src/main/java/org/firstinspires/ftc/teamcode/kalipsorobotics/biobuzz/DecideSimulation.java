package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.AutoDecider.HiveState;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Move;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.PartnerMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Pile;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PathPlanning;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

/**
 * A PRETEND match to test the NEW decision plan before the robot is finished. It uses NO robot
 * hardware and does NOT use the old RedTippingAuto.decide(): it plays the new rules in
 * AutoDecider.decide(), the same class the real auto can use. Nothing moves. No buttons:
 * press Start and it runs.
 *
 * THE NEW PLAN, in one sentence: decide() never guesses what the partner does. It only looks at the
 * Hive (which Cell is up, from the Limelight) and at our own robot, so one set of rules works for
 * every partner.
 *
 * We play 10 EDGE-CASE scenarios (see scenarioName), each TWICE:
 *   - camera ON : decide() is told which Cell is really up (the Limelight works)
 *   - camera OFF: decide() has to guess by counting balls (the Limelight does not work)
 * The goal is not to fix every case. It is to SEE what happens when something goes wrong.
 *
 * Things the partner does happen "during our Nth move", NOT at a clock time, because we do not
 * know how fast the robot really is.
 *
 * For every step it writes four lines: INPUTS, DECISION, PATH, RESULT.
 *   - logcat: search for "KLog_DecideSim" (everything is written once, right after Start)
 *   - Driver Hub: shows the same lines, one match at a time. It switches by itself.
 */
@TeleOp(name = "Decide Simulation", group = "biobuzz")
public class DecideSimulation extends LinearOpMode {

    // =============================================================================================
    // THE PRETEND WORLD
    // =============================================================================================

    // ----- Guesses for how long things take. Measure the real robot later. -----
    private static final double AUTO_SECONDS = 30;             // length of AUTO (manual 10.4)
    private static final double ROBOT_SPEED_MM_PER_SECOND = 1000; // GUESS: how fast the robot drives
    private static final double SECONDS_TO_PICK_UP = 2;        // GUESS: pick up the balls at a pile
    private static final double SECONDS_TO_SHOOT = 2;          // GUESS: shoot all the balls we hold

    // ----- Points (manual Table 10-2) -----
    private static final int POINTS_PER_TIP = 20;              // Hive Tip (the partner's tips count for us too)
    private static final int POINTS_FOR_LEAVE = 3;             // no longer touching the wall
    private static final int POINTS_FOR_PARK = 5;              // at least partly in the Loading Zone

    private static final int PREFILLED_BALLS_PER_PILE = 4;     // manual 10.3.1: 4 pollen in each pile
    private static final int MOST_BALLS_WE_CAN_HOLD = 4;       // manual G407
    private static final int NECTAR_IN_CELL_A_AT_START = 3;    // manual 10.3.1: the up Cell starts with 3 nectar

    // ----- Places on the field (mm, red alliance frame, same as RedTippingAuto) -----
    private static final Point START_POINT = new Point(171.5 + 1218, 3658 - 171.5);
    /** PLACEHOLDER: a spot inside the Loading Zone. */
    private static final Point PARK_POINT = new Point(100, 108);
    /** PLACEHOLDER: a loose ball on OUR side, and one on the OPPONENT's side (we must ignore that one). */
    private static final Point LOOSE_BALL_OUR_SIDE = new Point(800, 1800);
    private static final Point LOOSE_BALL_OPPONENT_SIDE = new Point(2800, 1800);
    /**
     * PLACEHOLDER: how far from a flower's center the robot stops. It must be more than half the
     * flower (63 mm) plus the obstacle buffer (350 mm) or the path planner says there is no path.
     */
    private static final double FLOWER_STAND_OFF_MM = 450;

    /** How long the Driver Hub shows one match before switching to the next. */
    private static final double SECONDS_PER_SCREEN = 15;

    // The path planner. forFieldAuto = also treats the opponent's half of the field as blocked.
    private static final double OBSTACLE_BUFFER_MM = 350;
    // One planner for a still Hive and one for a tipping Hive (the space under it is blocked).
    private final PathPlanning plannerHiveStill =
            PathPlanning.forFieldAuto(OBSTACLE_BUFFER_MM, RobotFootprint.WIDTH_MM / 2, false);
    private final PathPlanning plannerHiveTipping =
            PathPlanning.forFieldAuto(OBSTACLE_BUFFER_MM, RobotFootprint.WIDTH_MM / 2, true);

    /** The planner that matches the Hive right now. */
    private PathPlanning pathPlanner() {
        return tippedDuringLastMove ? plannerHiveTipping : plannerHiveStill;
    }

    // ----- The edge-case scenarios -----

    /** Number of different scenarios. */
    static final int SCENARIO_COUNT = 10;

    /** A short name for each scenario. "Move N" means our Nth move of the match. */
    static String scenarioName(int scenario) {
        switch (scenario) {
            case 0:  return "0 Baseline: the partner does nothing";
            case 1:  return "1 The partner tips the Hive while we drive to our 3rd move (a shot)";
            case 2:  return "2 The partner shoots into Cell A before our first shot lands";
            case 3:  return "3 The partner takes the Garden during our 1st move";
            case 4:  return "4 The partner takes the Garden AND Flower A during our 1st move";
            case 5:  return "5 Our 5th move (a shot) misses: only 2 of 4 balls land";
            case 6:  return "6 The partner tips the Hive twice in a row (during our 3rd and 4th moves)";
            case 7:  return "7 The Garden only has 2 balls (a partial load)";
            case 8:  return "8 No piles left, loose balls: one on our side, one on the opponent's side";
            default: return "9 The camera is lost during our 2nd to 4th moves, then comes back";
        }
    }

    /** One thing the partner does during our Nth move: shoot some balls into the up Cell, or empty a pile. */
    private static class PartnerEvent {
        final int duringStep;
        final int ballsShot;      // balls shot into whichever Cell is up (0 if this event takes a pile)
        final Pile pileTaken;     // the pile the partner empties (null if this event is a shot)

        PartnerEvent(int duringStep, int ballsShot, Pile pileTaken) {
            this.duringStep = duringStep;
            this.ballsShot = ballsShot;
            this.pileTaken = pileTaken;
        }
    }

    // ----- Settings of the current scenario (set in startMatch) -----
    private List<PartnerEvent> partnerEvents;
    private int missOnStep;                // our shot on this move misses (0 = none)
    private static final int BALLS_THAT_LAND_ON_A_MISS = 2;
    private int gardenBalls;               // how many balls the Garden really has
    private List<Point> looseBalls;        // loose balls lying on the field
    private int cameraLostFromStep;        // the camera cannot see from this move... (0 = never)
    private int cameraLostToStep;          // ...to this move

    // ----- THE TRUTH: what really happens on the pretend field -----
    private double secondsLeft;
    private Point robotPosition;
    private int ballsHeld;
    private Cell upCell;                   // the Cell that REALLY is up
    private int ballsInCellA;
    private int ballsInCellB;
    private int tipsDone;                  // tips that really happened (ours and the partner's)
    private int points;
    private boolean tippedDuringLastMove;  // the Hive moved during our last move, so it is TIPPING now
    private Set<Pile> pilesReallyThere;
    private boolean hasLeftTheWall;
    private boolean hasParked;
    private boolean matchIsOver;
    private int stepNumber;

    // ----- WHAT OUR ROBOT BELIEVES: all decide() is allowed to use -----
    private boolean cameraWorks;           // the Limelight works in this run
    private boolean cameraSeesNow;         // ...and it can see on this move (not lost)
    private final PartnerMode partnerMode = PartnerMode.NOT_SOLO; // the strong partner
    private Cell guessedUpCell;            // our guess from counting (used when the camera cannot see)
    private int guessedTipsDone;
    private int ballsWeShotIntoUpCell;
    private Set<Pile> pilesWeThinkHaveBalls;

    /** The lines of the match being played right now. (Not private: the laptop test reads it.) */
    List<String> currentReport;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press Start. This runs pretend matches, no robot movement.");
        telemetry.update();
        waitForStart();

        // Play every scenario twice (camera on, camera off) and keep each match's lines.
        List<List<String>> reports = new ArrayList<>();
        for (int scenario = 0; scenario < SCENARIO_COUNT; scenario++) {
            for (boolean camera : new boolean[]{true, false}) {
                currentReport = new ArrayList<>();
                playMatch(scenario, camera);
                reports.add(currentReport);
            }
        }

        // Show one match at a time on the Driver Hub, switching by itself.
        while (opModeIsActive()) {
            int screenNumber = (int) (getRuntime() / SECONDS_PER_SCREEN) % reports.size();
            telemetry.addLine("Screen " + (screenNumber + 1) + " of " + reports.size()
                    + " (switches every " + (int) SECONDS_PER_SCREEN + " s)");
            for (String line : reports.get(screenNumber)) {
                telemetry.addLine(line);
            }
            telemetry.update();
        }
    }

    /** Writes one line to logcat AND to the current report (which the Driver Hub shows). */
    private void say(String line) {
        currentReport.add(line);
        KLog.d("DecideSim", () -> line);
    }

    /** Plays a whole pretend match from the start to the end. (Not private: the laptop test calls it.) */
    void playMatch(int scenario, boolean cameraIsOn) {
        startMatch(scenario, cameraIsOn);
        say("===== " + scenarioName(scenario) + "  |  Limelight " + (cameraIsOn ? "ON" : "OFF (counting)")
                + " =====");

        while (!matchIsOver) {
            doOneStep();
        }

        say("===== FINAL: " + points + " AUTO points, " + tipsDone + " tips, "
                + (hasParked ? "parked" : "NOT parked") + ", " + round(secondsLeft) + "s left =====");
    }

    /** Puts the pretend match at the very start and sets up the scenario. */
    private void startMatch(int scenario, boolean cameraIsOn) {
        cameraWorks = cameraIsOn;

        // Normal world first...
        partnerEvents = new ArrayList<>();
        missOnStep = 0;
        gardenBalls = PREFILLED_BALLS_PER_PILE;
        looseBalls = new ArrayList<>();
        cameraLostFromStep = 0;
        cameraLostToStep = 0;
        pilesReallyThere = EnumSet.allOf(Pile.class);
        pilesWeThinkHaveBalls = EnumSet.allOf(Pile.class);

        // ...then the one thing that goes differently in this scenario.
        switch (scenario) {
            case 1:
                partnerEvents.add(new PartnerEvent(3, 8, null));      // 8 balls always tip the up Cell
                break;
            case 2:
                partnerEvents.add(new PartnerEvent(1, 4, null));      // Cell A has 3 nectar, 4 more tips it
                break;
            case 3:
                partnerEvents.add(new PartnerEvent(1, 0, Pile.GARDEN));
                break;
            case 4:
                partnerEvents.add(new PartnerEvent(1, 0, Pile.GARDEN));
                partnerEvents.add(new PartnerEvent(1, 0, Pile.FLOWER_A));
                break;
            case 5:
                missOnStep = 5;
                break;
            case 6:
                partnerEvents.add(new PartnerEvent(3, 8, null));
                partnerEvents.add(new PartnerEvent(4, 8, null));
                break;
            case 7:
                gardenBalls = 2;
                break;
            case 8:
                pilesReallyThere.clear();
                pilesWeThinkHaveBalls.clear();                         // we already know the piles are gone
                looseBalls.add(LOOSE_BALL_OUR_SIDE);
                looseBalls.add(LOOSE_BALL_OPPONENT_SIDE);
                break;
            case 9:
                cameraLostFromStep = 2;
                cameraLostToStep = 4;
                partnerEvents.add(new PartnerEvent(3, 8, null));      // happens while the camera is blind
                break;
            default:
                break; // 0: baseline
        }

        secondsLeft = AUTO_SECONDS;
        robotPosition = START_POINT;
        ballsHeld = MOST_BALLS_WE_CAN_HOLD;   // our 4 pollen preload
        upCell = Cell.A;
        ballsInCellA = NECTAR_IN_CELL_A_AT_START;
        ballsInCellB = 0;
        tipsDone = 0;
        points = 0;
        tippedDuringLastMove = false;
        hasLeftTheWall = false;
        hasParked = false;
        matchIsOver = false;
        stepNumber = 0;

        guessedUpCell = Cell.A;
        guessedTipsDone = 0;
        ballsWeShotIntoUpCell = 0;
    }

    /** One decision: say the inputs, ask decide(), say where we go and the path, pretend to do it. */
    private void doOneStep() {
        stepNumber++;

        // 0. The camera (if it works and can see right now) tells us the real Hive state.
        boolean cameraIsLostNow = (stepNumber >= cameraLostFromStep && stepNumber <= cameraLostToStep
                && cameraLostFromStep > 0);
        cameraSeesNow = cameraWorks && !cameraIsLostNow;
        HiveState seenByCamera = cameraSeesNow ? readHiveState() : HiveState.UNKNOWN;
        if (cameraSeesNow) {
            syncGuessWithTruth();
        }
        Cell upCellForDecide = AutoDecider.resolveUpCell(seenByCamera, guessedUpCell);
        boolean hiveIsTipping = (seenByCamera == HiveState.TIPPING);
        int ballsWeThinkAreInUpCell = ballsWeShotIntoUpCell
                + ((upCellForDecide == Cell.A && guessedTipsDone == 0) ? NECTAR_IN_CELL_A_AT_START : 0);
        boolean atShootPoint = Point.distance(robotPosition, launchPointOf(upCellForDecide)) < 1;
        int looseBallsSeen = countLooseBallsOnOurSide();
        double secondsToPark = secondsToDrive(robotPosition, PARK_POINT);
        double secondsForShotThenPark = secondsForShotThenPark(upCellForDecide);
        double secondsForTripThenPark = secondsForTripThenPark(upCellForDecide);

        // 1. INPUTS: exactly what decide() is about to be given.
        String hiveText = cameraSeesNow ? seenByCamera.toString()
                : (cameraWorks ? "camera LOST, we guess " : "camera off, we guess ") + guessedUpCell + " is up";
        say("#" + stepNumber + " INPUTS: secondsLeft=" + round(secondsLeft) + " toPark=" + round(secondsToPark)
                + " shotThenPark=" + round(secondsForShotThenPark) + " tripThenPark=" + round(secondsForTripThenPark)
                + " ballsHeld=" + ballsHeld + " atShootPoint=" + atShootPoint + " hive=" + hiveText
                + " tipsDone=" + guessedTipsDone + " pilesLeft=" + pilesWeThinkHaveBalls
                + " looseBallsSeen=" + looseBallsSeen + " robotAt=" + show(robotPosition));

        // 2. DECISION: the one move decide() picks.
        Move move = AutoDecider.decide(partnerMode, secondsLeft, secondsToPark, secondsForShotThenPark,
                secondsForTripThenPark, ballsHeld, atShootPoint, upCellForDecide, hiveIsTipping,
                ballsWeThinkAreInUpCell, guessedTipsDone, pilesWeThinkHaveBalls, looseBallsSeen);
        Point target = targetFor(move);
        String whereText = (target == null) ? "stay at " + show(robotPosition) : "go to " + show(target);
        say("   DECISION: " + move + " -> " + whereText + " (" + describe(move) + ")");

        // 3. PATH: the waypoints to drive there, and how long that takes.
        double secondsDriving = 0;
        boolean alreadyThere = (target == null) || Point.distance(robotPosition, target) < 1;
        if (alreadyThere) {
            say("   PATH: already there, no driving");
        } else {
            List<Point> waypoints = pathPlanner().plan(robotPosition, target);
            if (waypoints == null) {
                say("   PATH: NO PATH from " + show(robotPosition) + " to " + show(target) + ". Match over.");
                matchIsOver = true;
                return;
            }
            double distance = lengthOfPath(robotPosition, waypoints);
            secondsDriving = distance / ROBOT_SPEED_MM_PER_SECOND;
            say("   PATH: " + show(robotPosition) + " -> " + showAll(waypoints) + " = "
                    + Math.round(distance) + " mm, " + round(secondsDriving) + "s");
        }

        // 4. RESULT: pretend to finish the move.
        say("   RESULT: " + pretendToFinish(move, target, secondsDriving));
    }

    // ----- What the camera sees and what our robot believes -----

    /** What the Limelight would see: the REAL Hive state. */
    private HiveState readHiveState() {
        if (tippedDuringLastMove) {
            return HiveState.TIPPING;
        }
        return (upCell == Cell.A) ? HiveState.A_UP : HiveState.B_UP;
    }

    /** When the camera can see, our guess is fixed to the truth. If the Cell changed, our shot count starts over. */
    private void syncGuessWithTruth() {
        if (guessedUpCell != upCell || guessedTipsDone != tipsDone) {
            ballsWeShotIntoUpCell = 0;
        }
        guessedUpCell = upCell;
        guessedTipsDone = tipsDone;
    }

    /** After one of OUR shots we update our guess: if our count reaches the tip weight, we think it tipped. */
    private void updateGuessAfterOurShot(int ballsShot) {
        ballsWeShotIntoUpCell += ballsShot;
        int ballsWeThinkAreInUpCell = ballsWeShotIntoUpCell
                + ((guessedUpCell == Cell.A && guessedTipsDone == 0) ? NECTAR_IN_CELL_A_AT_START : 0);
        if (ballsWeThinkAreInUpCell >= RedTippingAuto.ballsNeededToTip(guessedTipsDone)) {
            guessedTipsDone++;
            guessedUpCell = (guessedUpCell == Cell.A) ? Cell.B : Cell.A;
            ballsWeShotIntoUpCell = 0;
        }
    }

    /** Loose balls the camera can see that are on OUR side. Balls on the opponent's side are ignored. */
    private int countLooseBallsOnOurSide() {
        int count = 0;
        for (Point ball : looseBalls) {
            if (RedTippingAuto.isOnOurSide(ball.getX())) {
                count++;
            }
        }
        return count;
    }

    private Point firstLooseBallOnOurSide() {
        for (Point ball : looseBalls) {
            if (RedTippingAuto.isOnOurSide(ball.getX())) {
                return ball;
            }
        }
        return null;
    }

    // ----- Targets, descriptions and drive times -----

    private Point launchPointOf(Cell cell) {
        return (cell == Cell.A) ? FieldConfig.aLaunchPoint : FieldConfig.bLaunchPoint;
    }

    private Point pilePointOf(Pile pile) {
        switch (pile) {
            case GARDEN:   return FieldConfig.gardenPoint;
            case FLOWER_A: return standOffFromFlower(FieldConfig.aFlowerPoint, +1, 0);
            default:       return standOffFromFlower(FieldConfig.bFlowerPoint, 0, +1);
        }
    }

    /** The place on the field each move drives to. null = the move does not drive anywhere. */
    private Point targetFor(Move move) {
        switch (move) {
            case SHOOT_A:        return launchPointOf(Cell.A);
            case SHOOT_B:        return launchPointOf(Cell.B);
            case GET_GARDEN:     return pilePointOf(Pile.GARDEN);
            case GET_FLOWER_A:   return pilePointOf(Pile.FLOWER_A);
            case GET_FLOWER_B:   return pilePointOf(Pile.FLOWER_B);
            case GET_SEEN_BALLS: return firstLooseBallOnOurSide();
            case PARK:           return PARK_POINT;
            default:             return null; // WAIT
        }
    }

    /**
     * A flower is an obstacle, so the robot cannot drive to its center. We stop a little way
     * from it, on the side away from the wall. (directionX, directionY) say which side: +1 or 0.
     */
    private static Point standOffFromFlower(Point flower, int directionX, int directionY) {
        return new Point(flower.getX() + directionX * FLOWER_STAND_OFF_MM,
                flower.getY() + directionY * FLOWER_STAND_OFF_MM);
    }

    /** A short plain-words description of what the move does. */
    private String describe(Move move) {
        switch (move) {
            case SHOOT_A:        return "drive to the A launch point and shoot into Cell A";
            case SHOOT_B:        return "drive to the B launch point and shoot into Cell B";
            case GET_GARDEN:     return "drive to the Garden and pick up its balls";
            case GET_FLOWER_A:   return "drive to Flower A and pick up its balls";
            case GET_FLOWER_B:   return "drive to Flower B and pick up its balls";
            case GET_SEEN_BALLS: return "drive to a loose ball the camera sees";
            case PARK:           return "drive to the Loading Zone and stop";
            default:             return "wait for the Hive to stop moving";
        }
    }

    /** Seconds the drive from one point to another would take (path length / speed). */
    private double secondsToDrive(Point from, Point to) {
        if (Point.distance(from, to) < 1) {
            return 0;
        }
        List<Point> waypoints = pathPlanner().plan(from, to);
        if (waypoints == null) {
            return 999;
        }
        return lengthOfPath(from, waypoints) / ROBOT_SPEED_MM_PER_SECOND;
    }

    /** Drive to the up Cell's launch point, shoot, then drive from there to the Loading Zone. */
    private double secondsForShotThenPark(Cell upCellToShoot) {
        Point launchPoint = launchPointOf(upCellToShoot);
        return secondsToDrive(robotPosition, launchPoint) + SECONDS_TO_SHOOT
                + secondsToDrive(launchPoint, PARK_POINT);
    }

    /** Drive to the next pile, pick up, drive to the up Cell's launch point, shoot, then park. 0 if no pile. */
    private double secondsForTripThenPark(Cell upCellToShoot) {
        Pile pile = AutoDecider.nextPile(partnerMode, pilesWeThinkHaveBalls);
        if (pile == null) {
            return 0;
        }
        Point launchPoint = launchPointOf(upCellToShoot);
        return secondsToDrive(robotPosition, pilePointOf(pile)) + SECONDS_TO_PICK_UP
                + secondsToDrive(pilePointOf(pile), launchPoint) + SECONDS_TO_SHOOT
                + secondsToDrive(launchPoint, PARK_POINT);
    }

    // ----- Pretending to do the move -----

    /**
     * Pretends to finish the move. Changes time, position, balls, tips and points, and returns a
     * sentence that says what happened. If there is not enough time to finish the move, the match
     * ends and nothing from that move counts.
     */
    private String pretendToFinish(Move move, Point target, double secondsDriving) {
        double secondsNeeded = secondsDriving;
        if (move == Move.SHOOT_A || move == Move.SHOOT_B) {
            secondsNeeded += SECONDS_TO_SHOOT;
        } else if (move == Move.GET_GARDEN || move == Move.GET_FLOWER_A || move == Move.GET_FLOWER_B
                || move == Move.GET_SEEN_BALLS) {
            secondsNeeded += SECONDS_TO_PICK_UP;
        }

        // Not enough time? The move never finishes, so it earns nothing.
        if (secondsNeeded > secondsLeft) {
            matchIsOver = true;
            return "NOT ENOUGH TIME (needs " + round(secondsNeeded) + "s, only " + round(secondsLeft)
                    + "s left). Nothing counts. Match over.";
        }
        secondsLeft -= secondsNeeded;
        tippedDuringLastMove = false; // from here on, a tip means "the Hive moved during THIS move"

        // Driving anywhere for the first time means we left the wall.
        String leaveText = "";
        if (secondsDriving > 0 && !hasLeftTheWall) {
            hasLeftTheWall = true;
            points += POINTS_FOR_LEAVE;
            leaveText = " +" + POINTS_FOR_LEAVE + " LEAVE.";
        }
        if (target != null) {
            robotPosition = target;
        }

        // The partner keeps doing things while we drive. Find out what happened.
        String partnerText = letPartnerAct();

        String whatHappened;
        switch (move) {
            case SHOOT_A:      whatHappened = shootBalls(Cell.A); break;
            case SHOOT_B:      whatHappened = shootBalls(Cell.B); break;
            case GET_GARDEN:   whatHappened = pickUpFromPile(Pile.GARDEN); break;
            case GET_FLOWER_A: whatHappened = pickUpFromPile(Pile.FLOWER_A); break;
            case GET_FLOWER_B: whatHappened = pickUpFromPile(Pile.FLOWER_B); break;
            case GET_SEEN_BALLS:
                looseBalls.remove(target);
                whatHappened = takeBalls(1);
                break;
            case PARK:
                points += POINTS_FOR_PARK;
                hasParked = true;
                matchIsOver = true;
                whatHappened = "parked, +" + POINTS_FOR_PARK + " PARK. Match over.";
                break;
            default:
                whatHappened = "waited for the Hive to settle.";
                break;
        }
        return whatHappened + partnerText + leaveText + " (" + round(secondsLeft) + "s left, " + points + " points)";
    }

    /** Picks up a pile. If the partner already took it, we get nothing and cross it off our list. */
    private String pickUpFromPile(Pile pile) {
        pilesWeThinkHaveBalls.remove(pile);
        boolean pileHasBalls = pilesReallyThere.remove(pile);
        if (!pileHasBalls) {
            return "the pile was EMPTY (the partner took it), picked up 0 balls, crossed it off the list.";
        }
        return takeBalls(pile == Pile.GARDEN ? gardenBalls : PREFILLED_BALLS_PER_PILE);
    }

    /** Puts balls into the robot (we can never hold more than 4). */
    private String takeBalls(int balls) {
        int taken = Math.min(balls, MOST_BALLS_WE_CAN_HOLD - ballsHeld);
        ballsHeld += taken;
        return "picked up " + taken + " balls, now holding " + ballsHeld + ".";
    }

    /**
     * Shoots every ball we hold at the Cell we drove to. Before firing, a camera that can see
     * checks again that this Cell is STILL up: the partner may have tipped the Hive while we drove.
     */
    private String shootBalls(Cell cellWeAimAt) {
        if (cameraSeesNow && upCell != cellWeAimAt) {
            return "re-checked the Hive: Cell " + cellWeAimAt + " is not up any more, shot CANCELLED, still holding "
                    + ballsHeld + " balls.";
        }

        int ballsShot = ballsHeld;
        ballsHeld = 0;
        updateGuessAfterOurShot(ballsShot);

        if (upCell != cellWeAimAt) {
            // We shot a Cell that was DOWN, so the balls do not count.
            return "shot " + ballsShot + " into Cell " + cellWeAimAt + " but it was DOWN: balls WASTED.";
        }
        boolean thisShotMisses = (stepNumber == missOnStep);
        int ballsThatLand = thisShotMisses ? Math.min(ballsShot, BALLS_THAT_LAND_ON_A_MISS) : ballsShot;
        String missText = thisShotMisses ? " (MISS: only " + ballsThatLand + " landed)" : "";
        return "shot " + ballsShot + " into Cell " + cellWeAimAt + "." + missText + addBallsToUpCell(ballsThatLand);
    }

    /**
     * Adds balls to the Cell that really is up. If the Cell reaches its tip weight, the Hive tips:
     * the Cell empties, the other Cell stands up, and the alliance scores. Returns a sentence about it.
     */
    private String addBallsToUpCell(int balls) {
        if (upCell == Cell.A) {
            ballsInCellA += balls;
        } else {
            ballsInCellB += balls;
        }
        int ballsInThisCell = (upCell == Cell.A) ? ballsInCellA : ballsInCellB;
        int ballsNeeded = RedTippingAuto.ballsNeededToTip(tipsDone);
        if (ballsInThisCell < ballsNeeded) {
            return " Cell " + upCell + " has " + ballsInThisCell + "/" + ballsNeeded + ", no tip yet.";
        }

        Cell cellThatTipped = upCell;
        if (upCell == Cell.A) {
            ballsInCellA = 0;
        } else {
            ballsInCellB = 0;
        }
        upCell = (upCell == Cell.A) ? Cell.B : Cell.A;
        tipsDone++;
        points += POINTS_PER_TIP;
        tippedDuringLastMove = true;
        return " HIVE TIPPED (Cell " + cellThatTipped + ")! +" + POINTS_PER_TIP + ". Cell " + upCell + " is now up.";
    }

    /** Lets the pretend partner do what it planned during THIS move. Returns a sentence about it. */
    private String letPartnerAct() {
        StringBuilder text = new StringBuilder();
        for (PartnerEvent event : partnerEvents) {
            if (event.duringStep != stepNumber) {
                continue;
            }
            if (event.pileTaken != null) {
                pilesReallyThere.remove(event.pileTaken);
                text.append(" [PARTNER took the ").append(event.pileTaken).append(".]");
            } else {
                text.append(" [PARTNER shot ").append(event.ballsShot).append(" balls into Cell ").append(upCell)
                        .append(".").append(addBallsToUpCell(event.ballsShot)).append("]");
            }
        }
        return text.toString();
    }

    /** Length in mm of: start -> first waypoint -> second waypoint -> ... */
    private double lengthOfPath(Point start, List<Point> waypoints) {
        double length = 0;
        Point previous = start;
        for (Point next : waypoints) {
            length += Point.distance(previous, next);
            previous = next;
        }
        return length;
    }

    // ----- Small helpers to print numbers and points in a short, readable way -----

    private static String round(double number) {
        return String.valueOf(Math.round(number * 10) / 10.0);
    }

    /** A point as (x, y) in whole mm. */
    private static String show(Point point) {
        return "(" + Math.round(point.getX()) + ", " + Math.round(point.getY()) + ")";
    }

    /** A list of points as (x, y) -> (x, y) -> ... */
    private static String showAll(List<Point> points) {
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < points.size(); i++) {
            if (i > 0) {
                text.append(" -> ");
            }
            text.append(show(points.get(i)));
        }
        return text.toString();
    }
}
