package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Move;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Pile;

/**
 * RedDecideAutoPartnerFirst for a STRONG partner (NOT SOLO).
 *
 * Each robot stays strictly on its own half (split at the middle of the Hive): the partner works the A half, we work the B half.
 * Neither shoots nor collects on the other half, so we never need to sense the partner.
 * The partner shoots its preload into A, then takes Flower A, the Garden (on the A half) and the A spill, and shoots them into A.
 * So Flower A and the Garden are crossed off our list; we only have Flower B, and loose balls the camera sees on the B half.
 * Everything else is the same: we start at the B end, wait for Cell B to be up, and then RedDecideAuto decides every move.
 */
@Autonomous(name = "Red Decide Auto (not solo)", group = "biobuzz")
public class RedDecideAutoNotSolo extends RedDecideAutoPartnerFirst {

    private static final double PARTNER_TIPS_A_AT_SECONDS = 18.5;   // GUESS: the sim partner's second batch (shot from 14.8 s) tips A about now. We cannot see A from B's end yet
    private boolean partnerTippedA = false;

    @Override
    void afterStart() {
        super.afterStart();
        pilesLeft.remove(Pile.FLOWER_A);   // the partner takes Flower A
        pilesLeft.remove(Pile.GARDEN);     // and the Garden, which is on its half
        pendingSpills.remove(Cell.A);      // the A spill is on its half too
    }

    /** The A end is the partner's: its spill is not ours to sweep. */
    @Override
    boolean ownsSpillOf(Cell tipped) {
        return tipped == Cell.B;
    }

    /** We only pick up loose balls on the B half (y below the middle of the Hive). */
    @Override
    boolean wantsBall(BallInformation ball) {
        return super.wantsBall(ball) && ball.y < FieldConfig.MIDLINE_X;   // the field is square, so its middle line is y = MIDLINE_X too
    }

    /**
     * decide() may say "shoot A" when A is the Cell that is up. A is the partner's: keep our balls and wait until B stands up again.
     * Nothing in this auto sees the partner's tip, so after PARTNER_TIPS_A_AT_SECONDS we believe A tipped and B is up.
     */
    @Override
    Move limitMove(Move move) {
        if (!partnerTippedA && upCell == Cell.A && getRuntime() >= PARTNER_TIPS_A_AT_SECONDS) {
            partnerTippedA = true;
            tipsDone++;
            upCell = Cell.B;
            ballsInCellA = 0;
            return Move.WAIT;   // decide again next time with the new belief
        }
        return (move == Move.SHOOT_A) ? Move.WAIT : move;
    }
}
