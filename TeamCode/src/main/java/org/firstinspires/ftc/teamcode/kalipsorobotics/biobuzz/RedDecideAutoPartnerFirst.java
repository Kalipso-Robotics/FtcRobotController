package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Move;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Pile;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData.CellState;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * RedDecideAuto for when the PARTNER shoots first (solo preload, the partner robot).
 *
 * The partner starts at the A end, where Cell A is up, and shoots its 4 preload into it. That tips A, and Cell B stands up.
 * We start at the B end ("the down side"), WAIT there until Cell B is up, then shoot our preload into it.
 * Everything after that is exactly RedDecideAuto: it decides every move the same way.
 *
 * It only changes the start, and what we believe at the start: "A already tipped, B is up and empty".
 * The numbers marked GUESS are unmeasured.
 */
@Autonomous(name = "Red Decide Auto (partner first)", group = "biobuzz")
public class RedDecideAutoPartnerFirst extends RedDecideAuto {

    private static final double START_X_MM = 1500;                  // GUESS: far enough from Flower B (x 1155-1283) that our body does not touch it
    private static final double CELL_B_UP_AT_SECONDS = 4.7;         // GUESS: the Hive finishes tipping about 3 s after the partner's last ball (sim: last ball 1.65 s, up at about 4.65 s)

    @Override
    Point startPoint() {
        return new Point(START_X_MM, RobotFootprint.BACK_MM);       // the back touches the wall at y = 0
    }

    @Override
    double startHeadingRad() {
        return Math.toRadians(90);                                  // the front points up the field, toward the Hive
    }

    @Override
    Point firstTurretTarget() {
        return ShooterConfig.TARGET_POINT_2;                            // the first Cell WE shoot is B
    }

    /**
     * The partner is shooting into A right now. Believe the result: A tipped, B is up and empty.
     * The balls of the tipped A Cell spill on the A side, so A goes on the spill list.
     * Then do not move until we would arrive at the B shooting spot about when B is up. Shooting earlier sends the balls into the wrong Cell.
     */
    @Override
    void afterStart() {
        tipsDone = 1;
        upCell = Cell.B;
        ballsInCellA = 0;
        ballsInCellB = 0;
        pendingSpills.add(Cell.A);
        double secondsToDrive = driveSeconds(robotPoint(), launchPointOf(Cell.B));
        decideNotBeforeSeconds = Math.max(0, CELL_B_UP_AT_SECONDS - secondsToDrive);
        hiveSettledAtSeconds = CELL_B_UP_AT_SECONDS;   // the same wait, now enforced at the shot even if we drive faster
        KLog.d("Decide", () -> "partner first: B is up at " + CELL_B_UP_AT_SECONDS + " s, so we start driving at " + decideNotBeforeSeconds + " s");
    }

    /**
     * We start at the B end, so the nearest pile is Flower B, right behind the start. Take it before the Garden or Flower A
     * (RedDecideAuto goes to the Garden first, which is a long drive to the far side).
     */
    @Override
    Move limitMove(Move move) {
        // A Cell has just tipped (tip 2 = B, tip 3 = A) and its balls are still coming down. Stand at the viewpoint beside the spot where they fall,
        // and do not leave for the first one or two we see: that loses the big group. Go when a group of 3 or more is seen,
        // or when the spill had time to land and nothing came. While we wait, no pile is worth leaving the viewpoint for.
        boolean spillPhase = (tipsDone == 2 || tipsDone == 3) && ballsHeld == 0;
        if (spillPhase) {
            boolean bigGroupSeen = seenCluster != null && seenCluster.size() >= MIN_SPILL_GROUP;
            boolean spillStillComing = getRuntime() < hiveSettledAtSeconds + SPILL_WAIT_SECONDS;
            if (bigGroupSeen) {
                return Move.GET_SEEN_BALLS;   // run through the balls we see, instead of a pile
            }
            if (spillStillComing && move != Move.PARK) {
                return Move.WAIT;
            }
        }
        boolean wantsOtherPile = (move == Move.GET_GARDEN || move == Move.GET_FLOWER_A);
        if (wantsOtherPile && pilesLeft.contains(Pile.FLOWER_B)) {
            return Move.GET_FLOWER_B;
        }
        return move;
    }

    private static final double SPILL_WAIT_SECONDS = 2.0;   // GUESS: how long after the Hive is down the spill takes to reach the floor
    private static final int MIN_SPILL_GROUP = 3;           // a group this big is the real spill: go

    // After a tip the Cell that went down dumps its balls on its own end of the field, in front of the Cell (x about 1300-1750).
    // We stand beside that spot, on our half, and look at it. GUESSES: tune from the sim.
    private static final Point VIEWPOINT_B_SPILL = new Point(950, 1050);     // tip 2: B went down, its balls fall near (1524, 1000), on the -y end
    private static final Point LOOK_TOWARD_B_SPILL = new Point(1524, 800);
    private static final Point VIEWPOINT_A_SPILL = new Point(950, 2608);      // tip 3: A went down, the same spot mirrored on the +y end
    private static final Point LOOK_TOWARD_A_SPILL = new Point(1524, 2857);

    /** After the tip that just happened (tip 2 = B, tip 3 = A), go and look at where its balls fall before anything else. */
    @Override
    boolean lookFirstAfterShot() {
        return tipsDone == 2 || tipsDone == 3;
    }

    @Override
    Point viewpointOfUpEnd() {
        if (tipsDone == 2) {
            return VIEWPOINT_B_SPILL;
        }
        return (tipsDone >= 3) ? VIEWPOINT_A_SPILL : super.viewpointOfUpEnd();
    }

    @Override
    Point lookTowardOfUpEnd() {
        if (tipsDone == 2) {
            return LOOK_TOWARD_B_SPILL;
        }
        return (tipsDone >= 3) ? LOOK_TOWARD_A_SPILL : super.lookTowardOfUpEnd();
    }

    /** Tip 2: only balls on the -y half (B's spill). From tip 3 on: only the +y half (A's spill). Never balls behind us. */
    @Override
    boolean wantsBall(BallInformation ball) {
        if (tipsDone == 2) {
            return RedTippingAuto.isOnOurSide(ball.x) && ball.y < FieldConfig.MIDLINE_X;
        }
        if (tipsDone >= 3) {
            return RedTippingAuto.isOnOurSide(ball.x) && ball.y > FieldConfig.MIDLINE_X;
        }
        return super.wantsBall(ball);
    }

    // ----- The Hive as the Limelight sees it: each Cell is UP, DOWN or TIPPING (null = not sensed, then we use our count) -----

    private static final double TIP_LAG_SECONDS = 5.0;   // GUESS: after our last ball a Hive may still need this long to tip; do not doubt our count before that

    /** State of one Cell. Cell B is the opposite of Cell A. null = nothing senses the Hive. */
    private static CellState stateOf(Cell cell) {
        CellState a = SharedData.getCellAState();
        if (a == null || a == CellState.TIPPING || cell == Cell.A) {
            return a;
        }
        return (a == CellState.UP) ? CellState.DOWN : CellState.UP;
    }

    /** Fire only at a Cell that is standing UP. A Cell that is TIPPING or DOWN sends the balls to the wrong place. */
    @Override
    boolean hiveReadyToShoot(Cell cell) {
        CellState state = stateOf(cell);
        return (state == null) ? super.hiveReadyToShoot(cell) : state == CellState.UP;
    }

    /**
     * The sensor wins over our count. If our last shot was counted as a tip but the Cell is still UP long after, it never tipped:
     * undo that tip. If the up Cell we believe in is DOWN while the other is UP (a tip we did not count), switch.
     */
    @Override
    void syncWithHive() {
        CellState a = SharedData.getCellAState();
        if (a == null) {
            super.syncWithHive(); // no Cell sensor: fall back to the Limelight
            return;
        }
        if (a == CellState.TIPPING) {
            return;
        }
        Cell sensedUp = (a == CellState.UP) ? Cell.A : Cell.B;
        if (sensedUp == upCell) {
            return;
        }
        boolean lastTipNeverHappened = lastShotCountedAsTip && lastShotCell == sensedUp;
        if (lastTipNeverHappened) {
            if (getRuntime() - lastShotAtSeconds < TIP_LAG_SECONDS) {
                return; // it may still be about to tip
            }
            tipsDone--;
            pendingSpills.remove(lastShotCell);
            setBallsInCell(lastShotCell, lastShotBallsBefore + lastShotBalls);
            lastShotCountedAsTip = false;
        } else {
            // a tip we did not count: the Cell we thought was up has tipped
            tipsDone++;
            pendingSpills.add(upCell);
            setBallsInCell(upCell, 0);
        }
        KLog.d("Decide", () -> "Hive sensor says " + sensedUp + " is up (we believed " + upCell + "): tipsDone=" + tipsDone);
        upCell = sensedUp;
    }

    private void setBallsInCell(Cell cell, int balls) {
        if (cell == Cell.A) {
            ballsInCellA = balls;
        } else {
            ballsInCellB = balls;
        }
    }
}
