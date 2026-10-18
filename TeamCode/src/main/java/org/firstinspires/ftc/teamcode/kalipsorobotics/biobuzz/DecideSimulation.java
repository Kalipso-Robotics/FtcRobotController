package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * A PRETEND match, so we can test the decision part of the auto before the robot is finished.
 * It uses NO robot hardware. Nothing moves. There are no buttons: press Start and it runs.
 *
 * What it does, once for EACH partner mode (SOLO_NOTHING, SOLO_PRELOAD, NOT_SOLO):
 *   1. Starts a pretend 30 second AUTO at the real start position, holding our 4 pollen preload.
 *   2. Asks RedTippingAuto.decide() "what should I do next?"  (all the thinking stays in decide())
 *   3. Turns the decision into a place to go (coordinates), asks PathPlanning for the waypoints
 *      to get there, and works out how long the drive takes from the path length.
 *   4. Pretends to finish the move: balls move around, Hives tip, points are added.
 *
 * For every step it writes four lines: INPUTS, DECISION, PATH, RESULT.
 *   - logcat: search for "KLog_DecideSim" (everything is written once, right after Start)
 *   - Driver Hub: shows the same lines, one partner mode at a time. It switches to the next
 *     partner mode by itself every few seconds.
 *
 * The speeds and times below are GUESSES, not measured. Change them to test other situations.
 */
@TeleOp(name = "Decide Simulation", group = "biobuzz")
public class DecideSimulation extends LinearOpMode {

    // ----- Guesses for how long things take. Measure the real robot later. -----
    private static final double AUTO_SECONDS = 30;             // length of AUTO (manual 10.4)
    private static final double ROBOT_SPEED_MM_PER_SECOND = 1000; // GUESS: how fast the robot drives
    private static final double SECONDS_TO_PICK_UP = 2;        // pick up the balls once we are at a pile
    private static final double SECONDS_TO_SHOOT = 2;          // shoot all the balls we hold
    /**
     * GUESS, NOT_SOLO only: after OUR tip, the partner needs this many seconds to shoot its preload,
     * fetch Flower B and tip Cell B. Then Cell A is up again for us. We cannot know this number
     * until we practice with the partner.
     */
    private static final double PARTNER_SECONDS_TO_TIP_B = 12;
    private static final double NO_PARTNER_TIP_PLANNED = -1;

    // ----- Points (manual Table 10-2) -----
    private static final int POINTS_PER_TIP = 20;              // Hive Tip
    private static final int POINTS_FOR_LEAVE = 3;             // no longer touching the wall
    private static final int POINTS_FOR_PARK = 5;              // at least partly in the Loading Zone

    private static final int PREFILLED_BALLS_PER_PILE = 4;     // manual 10.3.1: 4 pollen in each pile
    private static final int MOST_BALLS_WE_CAN_HOLD = 4;       // manual G407
    private static final int NECTAR_IN_UP_CELL_AT_START = 3;   // manual 10.3.1

    // ----- Places on the field (mm, red alliance frame, same as RedTippingAuto) -----
    /** Where the robot starts. Copied from the odometry start in RedTippingAuto. */
    private static final Point START_POINT = new Point(171.5 + 1218, 3658 - 171.5);
    /**
     * PLACEHOLDER: how far from a flower's center the robot stops. It must be more than half the
     * flower (63 mm) plus the obstacle buffer (350 mm) or the path planner says there is no path.
     */
    private static final double FLOWER_STAND_OFF_MM = 450;
    /** PLACEHOLDER: a spot inside the Loading Zone. RedTippingAuto's park path ends near here. */
    private static final Point PARK_POINT = new Point(100, 108);
    /** PLACEHOLDER: where a loose ball is, if the camera sees one. It is on OUR side of the field. */
    private static final Point LOOSE_BALL_POINT = new Point(800, 1800);

    /** Pretend the camera sees this many loose balls at the start. 0 = none. Try 2 to see GET_SEEN_BALLS. */
    private static final int LOOSE_BALLS_AT_START = 0;

    /** How long the Driver Hub shows one partner mode before switching to the next. */
    private static final double SECONDS_PER_SCREEN = 8;

    // The path planner. forFieldAuto = also treats the opponent's half of the field as blocked.
    // 350 is how far the robot center stays from obstacles (same number AutoToBallsPathingAction uses).
    private static final double OBSTACLE_BUFFER_MM = 350;
    private final PathPlanning pathPlanner =
            PathPlanning.forFieldAuto(OBSTACLE_BUFFER_MM, RobotFootprint.WIDTH_MM / 2);

    // ----- The pretend match. startMatch() sets all of these back to the start. -----
    private PartnerMode partnerMode;
    private double secondsLeft;
    private Point robotPosition;
    private int ballsHeld;
    private boolean atShootPoint;        // true = standing at the launch point of the UP Cell
    private Cell upCell;
    private int ballsInUpCell;
    private int tipsDone;
    private Set<Pile> pilesLeft;
    private int looseBallsSeen;
    private int points;
    private boolean hasLeftTheWall;
    private boolean matchIsOver;
    private int stepNumber;
    /** NOT_SOLO: the seconds-left value at which the partner tips Cell B. */
    private double partnerTipsBAtSecondsLeft;

    /** The lines of the match being played right now. */
    // (not private, so the laptop test in the same package can read the lines)
    List<String> currentReport;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press Start. This runs a pretend match, no robot movement.");
        telemetry.update();
        waitForStart();

        // Play one pretend match per partner mode and keep each one's lines.
        PartnerMode[] allModes = PartnerMode.values();
        List<List<String>> reports = new ArrayList<>();
        for (PartnerMode mode : allModes) {
            currentReport = new ArrayList<>();
            playWholeMatch(mode);
            reports.add(currentReport);
        }

        // Show one report at a time on the Driver Hub, switching by itself.
        while (opModeIsActive()) {
            int screenNumber = (int) (getRuntime() / SECONDS_PER_SCREEN) % allModes.length;
            telemetry.addLine("Screen " + (screenNumber + 1) + " of " + allModes.length
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

    /** Plays a whole pretend match from the start to the end, one decision at a time. */
    // (not private, so the laptop test in the same package can call it)
    void playWholeMatch(PartnerMode mode) {
        startMatch(mode);
        say("===== PARTNER MODE: " + partnerMode + " =====");

        while (!matchIsOver) {
            doOneStep();
        }

        say("===== FINAL: " + points + " AUTO points, " + tipsDone + " tips, "
                + round(secondsLeft) + "s left, partner " + partnerMode + " =====");
    }

    /** Puts the pretend match at the very start. */
    private void startMatch(PartnerMode mode) {
        partnerMode = mode;
        secondsLeft = AUTO_SECONDS;
        robotPosition = START_POINT;
        ballsHeld = MOST_BALLS_WE_CAN_HOLD;   // our 4 pollen preload
        atShootPoint = false;
        upCell = Cell.A;
        ballsInUpCell = NECTAR_IN_UP_CELL_AT_START;
        tipsDone = 0;
        pilesLeft = EnumSet.allOf(Pile.class);
        looseBallsSeen = LOOSE_BALLS_AT_START;
        points = 0;
        hasLeftTheWall = false;
        matchIsOver = false;
        stepNumber = 0;
        partnerTipsBAtSecondsLeft = NO_PARTNER_TIP_PLANNED;
    }

    /** One decision: say the inputs, ask decide(), say where we go and the path, pretend to do it. */
    private void doOneStep() {
        stepNumber++;

        // 1. INPUTS: exactly what decide() is about to be given.
        say("#" + stepNumber + " INPUTS: secondsLeft=" + round(secondsLeft) + " ballsHeld=" + ballsHeld
                + " atShootPoint=" + atShootPoint + " upCell=" + upCell + "(" + ballsInUpCell + "/"
                + RedTippingAuto.ballsNeededToTip(tipsDone) + " balls) tipsDone=" + tipsDone
                + " pilesLeft=" + pilesLeft + " looseBallsSeen=" + looseBallsSeen
                + " robotAt=" + show(robotPosition));

        // 2. DECISION: the one move decide() picks.
        Move move = RedTippingAuto.decide(partnerMode, secondsLeft, ballsHeld, atShootPoint,
                upCell, ballsInUpCell, tipsDone, pilesLeft, looseBallsSeen);
        Point target = targetFor(move);
        String whereText = (target == null) ? "stay at " + show(robotPosition) : "go to " + show(target);
        say("   DECISION: " + move + " -> " + whereText + " (" + describe(move) + ")");

        // 3. PATH: the waypoints to drive there, and how long that takes.
        List<Point> waypoints = new ArrayList<>();
        double secondsToDrive = 0;
        boolean alreadyThere = (target == null) || (isShoot(move) && atShootPoint);
        if (alreadyThere) {
            say("   PATH: already there, no driving");
        } else {
            waypoints = pathPlanner.plan(robotPosition, target);
            if (waypoints == null) {
                say("   PATH: NO PATH from " + show(robotPosition) + " to " + show(target) + ". Match over.");
                matchIsOver = true;
                return;
            }
            double distance = lengthOfPath(robotPosition, waypoints);
            secondsToDrive = distance / ROBOT_SPEED_MM_PER_SECOND;
            say("   PATH: " + show(robotPosition) + " -> " + showAll(waypoints) + " = "
                    + Math.round(distance) + " mm, " + round(secondsToDrive) + "s");
        }

        // 4. RESULT: pretend to finish the move.
        say("   RESULT: " + pretendToFinish(move, target, secondsToDrive));
    }

    /** The place on the field each move drives to. null = the move does not drive anywhere. */
    private Point targetFor(Move move) {
        switch (move) {
            case SHOOT_A:        return FieldConfig.aLaunchPoint;
            case SHOOT_B:        return FieldConfig.bLaunchPoint;
            case GET_GARDEN:     return FieldConfig.gardenPoint;
            case GET_FLOWER_A:   return standOffFromFlower(FieldConfig.aFlowerPoint, +1, 0);
            case GET_FLOWER_B:   return standOffFromFlower(FieldConfig.bFlowerPoint, 0, +1);
            case GET_SEEN_BALLS: return LOOSE_BALL_POINT;
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
            default:             return "wait near A and hold our balls until the partner tips Cell B";
        }
    }

    /**
     * Pretends to finish the move. Changes time, position, balls, tips and points, and returns a
     * sentence that says what happened. If there is not enough time to finish the move, the match
     * ends and nothing from that move counts.
     */
    private String pretendToFinish(Move move, Point target, double secondsToDrive) {
        // Time for the part that is not driving.
        double secondsNeeded = secondsToDrive;
        if (isShoot(move)) {
            secondsNeeded += SECONDS_TO_SHOOT;
        } else if (isPickUp(move)) {
            secondsNeeded += SECONDS_TO_PICK_UP;
        }

        if (move == Move.WAIT) {
            // Wait until the partner tips Cell B, or until the park time starts, whichever is first.
            double waitUntil = RedTippingAuto.PARK_START_SECONDS;
            boolean partnerTipComing = partnerTipsBAtSecondsLeft > waitUntil && partnerTipsBAtSecondsLeft < secondsLeft;
            if (partnerTipComing) {
                waitUntil = partnerTipsBAtSecondsLeft;
            }
            secondsNeeded = Math.max(0, secondsLeft - waitUntil);
        }

        // Not enough time? The move never finishes, so it earns nothing.
        if (secondsNeeded > secondsLeft) {
            matchIsOver = true;
            return "NOT ENOUGH TIME (needs " + round(secondsNeeded) + "s, only " + round(secondsLeft)
                    + "s left). Nothing counts. Match over.";
        }
        secondsLeft -= secondsNeeded;

        // Driving anywhere for the first time means we left the wall.
        String leaveText = "";
        if (secondsToDrive > 0 && !hasLeftTheWall) {
            hasLeftTheWall = true;
            points += POINTS_FOR_LEAVE;
            leaveText = " +" + POINTS_FOR_LEAVE + " LEAVE.";
        }
        if (target != null) {
            robotPosition = target;
        }

        String whatHappened;
        switch (move) {
            case SHOOT_A:
            case SHOOT_B:
                whatHappened = shootBalls();
                break;
            case GET_GARDEN:
                pilesLeft.remove(Pile.GARDEN);
                whatHappened = pickUpBalls(PREFILLED_BALLS_PER_PILE);
                break;
            case GET_FLOWER_A:
                pilesLeft.remove(Pile.FLOWER_A);
                whatHappened = pickUpBalls(PREFILLED_BALLS_PER_PILE);
                break;
            case GET_FLOWER_B:
                pilesLeft.remove(Pile.FLOWER_B);
                whatHappened = pickUpBalls(PREFILLED_BALLS_PER_PILE);
                break;
            case GET_SEEN_BALLS:
                int ballsPicked = Math.min(looseBallsSeen, MOST_BALLS_WE_CAN_HOLD);
                looseBallsSeen -= ballsPicked;
                whatHappened = pickUpBalls(ballsPicked);
                break;
            case PARK:
                points += POINTS_FOR_PARK;
                matchIsOver = true;
                whatHappened = "parked, +" + POINTS_FOR_PARK + " PARK. Match over.";
                break;
            default:
                whatHappened = "waited";
                break;
        }
        whatHappened += checkIfPartnerTipped();
        return whatHappened + leaveText + " (" + round(secondsLeft) + "s left, " + points + " points)";
    }

    private boolean isShoot(Move move) {
        return move == Move.SHOOT_A || move == Move.SHOOT_B;
    }

    private boolean isPickUp(Move move) {
        return move == Move.GET_GARDEN || move == Move.GET_FLOWER_A
                || move == Move.GET_FLOWER_B || move == Move.GET_SEEN_BALLS;
    }

    /** Picks up balls (we can never hold more than 4). We are no longer at a launch point. */
    private String pickUpBalls(int ballsFound) {
        int roomLeft = MOST_BALLS_WE_CAN_HOLD - ballsHeld;
        int ballsTaken = Math.min(ballsFound, roomLeft);
        ballsHeld += ballsTaken;
        atShootPoint = false;
        return "picked up " + ballsTaken + " balls, now holding " + ballsHeld + ".";
    }

    /** Shoots every ball we hold into the UP Cell. If that is enough weight, the Hive tips. */
    private String shootBalls() {
        int ballsShot = ballsHeld;
        ballsHeld = 0;
        atShootPoint = true;
        ballsInUpCell += ballsShot;

        int ballsNeeded = RedTippingAuto.ballsNeededToTip(tipsDone);
        if (ballsInUpCell < ballsNeeded) {
            return "shot " + ballsShot + " into Cell " + upCell + " (" + ballsInUpCell + "/" + ballsNeeded
                    + " balls), no tip yet.";
        }

        // The Hive tips: points, and the other Cell stands up.
        Cell cellThatTipped = upCell;
        tipsDone++;
        points += POINTS_PER_TIP;
        upCell = (upCell == Cell.A) ? Cell.B : Cell.A;
        ballsInUpCell = RedTippingAuto.partnerBallsInNewCell(partnerMode, tipsDone);
        atShootPoint = false; // we stand at the old Cell's launch point, but the NEW up Cell has its own
        if (partnerMode == PartnerMode.NOT_SOLO && upCell == Cell.B) {
            partnerTipsBAtSecondsLeft = secondsLeft - PARTNER_SECONDS_TO_TIP_B;
        }
        return "shot " + ballsShot + " into Cell " + cellThatTipped + ", HIVE TIPPED! +" + POINTS_PER_TIP
                + ". Cell " + upCell + " is now up with " + ballsInUpCell + " balls.";
    }

    /**
     * NOT_SOLO only: if the pretend partner has had enough time, it tips Cell B now. Cell A comes up
     * again with nothing in it, and the partner's tip scores for our alliance too.
     */
    private String checkIfPartnerTipped() {
        boolean partnerTipDue = (partnerMode == PartnerMode.NOT_SOLO) && (upCell == Cell.B)
                && partnerTipsBAtSecondsLeft != NO_PARTNER_TIP_PLANNED
                && secondsLeft <= partnerTipsBAtSecondsLeft;
        if (!partnerTipDue) {
            return "";
        }
        tipsDone++;
        points += POINTS_PER_TIP;
        upCell = Cell.A;
        ballsInUpCell = 0;
        partnerTipsBAtSecondsLeft = NO_PARTNER_TIP_PLANNED;
        // If we are standing at the A launch point, we are ready to shoot A.
        atShootPoint = Point.distance(robotPosition, FieldConfig.aLaunchPoint) < 1;
        return " PARTNER TIPPED Cell B, +" + POINTS_PER_TIP + ". Cell A is up with 0 balls.";
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
        if (point == null) {
            return "(nowhere)";
        }
        return "(" + Math.round(point.getX()) + ", " + Math.round(point.getY()) + ")";
    }

    /** A list of points as (x, y) -> (x, y) -> ... */
    private static String showAll(List<Point> points) {
        if (points.isEmpty()) {
            return "(no waypoints)";
        }
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
