package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Move;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.PartnerMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Pile;

import java.util.Set;

/**
 * THE BRAIN of the AUTO: decide() takes a few numbers in and gives back exactly ONE Move.
 *
 * It never touches the robot, so it can be used from any OpMode and tested on a laptop:
 *     Move next = AutoDecider.decide(mode, secondsLeft, ...);
 * The pretend matches in DecideSimulation use this same class.
 */
public class AutoDecider {

    /** What the Limelight sees on the Hive. UNKNOWN means "I cannot tell", so we fall back to counting. */
    public enum HiveState {
        A_UP,        // Cell A is standing up and settled
        B_UP,        // Cell B is standing up and settled
        TIPPING,     // the Hive is moving right now
        UNKNOWN      // the camera cannot read it
    }

    /** Extra safety seconds on top of every "does it fit before I must park?" check. */
    public static final double PARK_MARGIN_SECONDS = 0.5;

    /** Which Cell is up? If the camera can read the Hive, believe the camera. If not, use our counted guess. */
    public static Cell resolveUpCell(HiveState seenByCamera, Cell countedGuess) {
        if (seenByCamera == HiveState.A_UP) {
            return Cell.A;
        }
        if (seenByCamera == HiveState.B_UP) {
            return Cell.B;
        }
        return countedGuess;
    }

    /**
     * The first pile in our order that still has balls, or null if none is left. Every pile is worth
     * a visit: if the partner already emptied it, we find out for free and cross it off.
     */
    public static Pile nextPile(PartnerMode mode, Set<Pile> pilesLeft) {
        for (Pile pile : RedTippingAuto.pileOrder(mode)) {
            if (pilesLeft.contains(pile)) {
                return pile;
            }
        }
        return null;
    }

    /**
     * Picks the next move. The rules are checked from top to bottom, the FIRST one that fits wins:
     *
     *   1. Almost out of time -> PARK. "Almost" = the time left is no more than the time to drive to
     *                            the Loading Zone from HERE (plus a margin). Exception: if one more
     *                            shot is SURE to tip a Cell, take it first.
     *   2. Holding balls, Hive tipping -> WAIT (never shoot a moving Hive). If we are empty we do
     *                            not wait: we drive off to get more balls.
     *   3. Holding balls       -> shoot the UP Cell, but only if drive + shot + the drive to park fits.
     *   4. Empty, a pile left  -> get that pile, but only if pickup trip + shot + the drive to park fits.
     *   5. Empty, no pile, loose balls seen on OUR side -> get those.
     *   6. Otherwise           -> PARK.
     *
     * @param mode          what the partner does (only changes the order of the piles)
     * @param secondsLeft   seconds left in AUTO
     * @param secondsToPark drive time to the Loading Zone from where the robot is now
     * @param secondsForShotThenPark  drive to the up Cell's launch point + shoot + drive from there to park
     * @param secondsForTripThenPark  drive to the next pile + pick up + drive to the up Cell's launch
     *                      point + shoot + drive from there to park
     * @param ballsHeld     balls inside our robot (beam break)
     * @param atShootPoint  true if the robot already stands at the launch point of the up Cell
     * @param upCell        the Cell that is up (from the camera, or our guess if it cannot see)
     * @param hiveIsTipping true if the camera sees the Hive moving right now
     * @param ballsInUpCell our best guess of the balls inside the up Cell (only for the last-shot check)
     * @param tipsDone      how many tips already happened
     * @param pilesLeft     piles that still have balls (remove a pile when a pickup found nothing)
     * @param looseBallsSeen loose balls the camera sees on OUR side only (see RedTippingAuto.isOnOurSide)
     */
    public static Move decide(PartnerMode mode, double secondsLeft, double secondsToPark,
                       double secondsForShotThenPark, double secondsForTripThenPark, int ballsHeld,
                       boolean atShootPoint, Cell upCell, boolean hiveIsTipping, int ballsInUpCell,
                       int tipsDone, Set<Pile> pilesLeft, int looseBallsSeen) {

        boolean holdingBalls = (ballsHeld > 0);
        Move shootUpCell = (upCell == Cell.A) ? Move.SHOOT_A : Move.SHOOT_B;

        // Rule 1: almost out of time.
        boolean almostOutOfTime = (secondsLeft <= secondsToPark + PARK_MARGIN_SECONDS);
        if (almostOutOfTime) {
            boolean thisShotWillTip = (ballsInUpCell + ballsHeld >= RedTippingAuto.ballsNeededToTip(tipsDone));
            boolean sureTip = holdingBalls && atShootPoint && thisShotWillTip && !hiveIsTipping;
            return sureTip ? shootUpCell : Move.PARK;
        }

        // Rule 2 and 3: we have balls, so shoot them (if the Hive is still, and there is time).
        if (holdingBalls) {
            if (hiveIsTipping) {
                return Move.WAIT;
            }
            boolean timeForOneMoreShot = (secondsLeft > secondsForShotThenPark + PARK_MARGIN_SECONDS);
            return timeForOneMoreShot ? shootUpCell : Move.PARK;
        }

        // Rule 4 and 5: we are empty, so go get balls (if there is time for the whole trip).
        boolean timeForOneMoreTrip = (secondsLeft > secondsForTripThenPark + PARK_MARGIN_SECONDS);
        if (!timeForOneMoreTrip) {
            return Move.PARK;
        }
        Pile pile = nextPile(mode, pilesLeft);
        if (pile == Pile.GARDEN) {
            return Move.GET_GARDEN;
        }
        if (pile == Pile.FLOWER_A) {
            return Move.GET_FLOWER_A;
        }
        if (pile == Pile.FLOWER_B) {
            return Move.GET_FLOWER_B;
        }
        if (looseBallsSeen > 0) {
            return Move.GET_SEEN_BALLS;
        }

        // Rule 6: nothing left to do.
        return Move.PARK;
    }
}
