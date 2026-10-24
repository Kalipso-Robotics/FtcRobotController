package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Move;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.PartnerMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Pile;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.junit.Test;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

public class RedTippingAutoDecideTest {
    private static final double PLENTY_OF_TIME = 20;
    private static final Set<Pile> ALL_PILES = EnumSet.allOf(Pile.class);
    private static final Set<Pile> NO_PILES = EnumSet.noneOf(Pile.class);

    /** Empty robot, plenty of time, Cell A up, nothing seen: the simplest question to ask. */
    private static Move emptyRobot(PartnerMode mode, Set<Pile> piles, int seenBalls) {
        return RedTippingAuto.decide(mode, PLENTY_OF_TIME, 0, false, Cell.A, 0, 0, piles, seenBalls);
    }

    // ---- holding balls: shoot the UP Cell ----

    @Test
    public void holdingBalls_shootsTheUpCell() {
        assertEquals(Move.SHOOT_A, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING, PLENTY_OF_TIME,
                4, false, Cell.A, 3, 0, ALL_PILES, 0));
        assertEquals(Move.SHOOT_B, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING, PLENTY_OF_TIME,
                4, false, Cell.B, 0, 1, ALL_PILES, 0));
    }

    @Test
    public void holdingBalls_shootsEvenWithFewerThanATip() {
        assertEquals(Move.SHOOT_A, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING, PLENTY_OF_TIME,
                1, false, Cell.A, 0, 0, ALL_PILES, 0));
    }

    // ---- empty robot: go get the next pile ----

    @Test
    public void solo_pilesGoGardenThenFlowerBThenFlowerA() {
        for (PartnerMode mode : new PartnerMode[]{PartnerMode.SOLO_NOTHING, PartnerMode.SOLO_PRELOAD}) {
            Set<Pile> piles = EnumSet.allOf(Pile.class);
            assertEquals(Move.GET_GARDEN, emptyRobot(mode, piles, 0));
            piles.remove(Pile.GARDEN);
            assertEquals(Move.GET_FLOWER_B, emptyRobot(mode, piles, 0));
            piles.remove(Pile.FLOWER_B);
            assertEquals(Move.GET_FLOWER_A, emptyRobot(mode, piles, 0));
        }
    }

    @Test
    public void notSolo_pilesGoGardenThenFlowerA() {
        Set<Pile> piles = EnumSet.allOf(Pile.class);
        assertEquals(Move.GET_GARDEN, emptyRobot(PartnerMode.NOT_SOLO, piles, 5));
        piles.remove(Pile.GARDEN);
        assertEquals(Move.GET_FLOWER_A, emptyRobot(PartnerMode.NOT_SOLO, piles, 5));
    }

    @Test
    public void notSolo_flowerBOnlyIfBallsAreStillSeen() {
        Set<Pile> onlyB = EnumSet.of(Pile.FLOWER_B);
        assertEquals(Move.GET_FLOWER_B, emptyRobot(PartnerMode.NOT_SOLO, onlyB, 3));
        assertEquals(Move.PARK, emptyRobot(PartnerMode.NOT_SOLO, onlyB, 0));
    }

    @Test
    public void aPickupThatGotNothing_skipsToTheNextPile() {
        // the caller removes the Garden after a pickup with 0 balls
        Set<Pile> piles = EnumSet.of(Pile.FLOWER_B, Pile.FLOWER_A);
        assertEquals(Move.GET_FLOWER_B, emptyRobot(PartnerMode.SOLO_NOTHING, piles, 0));
    }

    // ---- piles gone ----

    @Test
    public void pilesGone_ballsSeen_getSeenBalls() {
        assertEquals(Move.GET_SEEN_BALLS, emptyRobot(PartnerMode.SOLO_NOTHING, NO_PILES, 2));
    }

    @Test
    public void pilesGone_nothingSeen_parks() {
        assertEquals(Move.PARK, emptyRobot(PartnerMode.SOLO_NOTHING, NO_PILES, 0));
    }

    // ---- the park window ----

    @Test
    public void justAfterParkTime_stillWorks() {
        assertEquals(Move.GET_GARDEN, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING,
                RedTippingAuto.PARK_START_SECONDS + 0.1, 0, false, Cell.A, 0, 0, ALL_PILES, 0));
    }

    @Test
    public void atParkTime_parks() {
        assertEquals(Move.PARK, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING,
                RedTippingAuto.PARK_START_SECONDS, 0, false, Cell.A, 0, 0, ALL_PILES, 0));
    }

    @Test
    public void parkWindow_holdingBallsAwayFromShootPoint_parks() {
        assertEquals(Move.PARK, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING, 2, 4, false,
                Cell.A, 3, 0, ALL_PILES, 0));
    }

    @Test
    public void parkWindow_atShootPointButShotCannotTip_parks() {
        // the Cell has its 3 nectar + we hold 3 = 6, but tip 1 needs 7
        assertEquals(Move.PARK, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING, 2, 3, true,
                Cell.A, 3, 0, ALL_PILES, 0));
    }

    @Test
    public void parkWindow_atShootPointAndShotWillTip_shootsFirst() {
        assertEquals(Move.SHOOT_A, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING, 2, 4, true,
                Cell.A, 3, 0, ALL_PILES, 0));
        // later tip: Cell B already has 5, we hold 3, 5 + 3 = 8
        assertEquals(Move.SHOOT_B, RedTippingAuto.decide(PartnerMode.SOLO_NOTHING, 2, 3, true,
                Cell.B, 5, 1, ALL_PILES, 0));
    }

    // ---- tip weights and the partner's preload ----

    @Test
    public void tipWeights_firstIsSevenThenEight() {
        assertEquals(7, RedTippingAuto.ballsNeededToTip(0));
        assertEquals(8, RedTippingAuto.ballsNeededToTip(1));
        assertEquals(8, RedTippingAuto.ballsNeededToTip(2));
    }

    @Test
    public void partnerPreload_onlyHelpsAfterTipOne_andNotInSoloNothing() {
        assertEquals(0, RedTippingAuto.partnerBallsInNewCell(PartnerMode.SOLO_PRELOAD, 0));
        assertEquals(4, RedTippingAuto.partnerBallsInNewCell(PartnerMode.SOLO_PRELOAD, 1));
        assertEquals(4, RedTippingAuto.partnerBallsInNewCell(PartnerMode.NOT_SOLO, 1));
        assertEquals(0, RedTippingAuto.partnerBallsInNewCell(PartnerMode.SOLO_NOTHING, 1));
    }

    @Test
    public void partnerPreload_makesTipTwoOneTripEarlier() {
        // Tip 2 needs 8. Solo-nothing: 8 more from us (two trips of 4).
        // With the partner's 4 waiting, only 4 more from us (one trip).
        int soloNeed = RedTippingAuto.ballsNeededToTip(1)
                - RedTippingAuto.partnerBallsInNewCell(PartnerMode.SOLO_NOTHING, 1);
        int withPartnerNeed = RedTippingAuto.ballsNeededToTip(1)
                - RedTippingAuto.partnerBallsInNewCell(PartnerMode.SOLO_PRELOAD, 1);
        assertEquals(8, soloNeed);
        assertEquals(4, withPartnerNeed);
    }

    @Test
    public void nextPile_noneLeft_isNull() {
        assertNull(RedTippingAuto.nextPile(PartnerMode.SOLO_NOTHING, NO_PILES, 3));
    }

    // ---- we never cross to the opponent's side in AUTO ----

    @Test
    public void ballsOnTheOpponentsSide_areNotOurs() {
        assertEquals(true, RedTippingAuto.isOnOurSide(FieldConfig.MIDLINE_X - 1));
        assertEquals(false, RedTippingAuto.isOnOurSide(FieldConfig.MIDLINE_X + 1));
    }

    @Test
    public void allOurPilesAndLaunchPoints_areOnOurSide() {
        assertEquals(true, RedTippingAuto.isOnOurSide(FieldConfig.gardenPoint.getX()));
        assertEquals(true, RedTippingAuto.isOnOurSide(FieldConfig.aFlowerPoint.getX()));
        assertEquals(true, RedTippingAuto.isOnOurSide(FieldConfig.bFlowerPoint.getX()));
        assertEquals(true, RedTippingAuto.isOnOurSide(FieldConfig.aLaunchPoint.getX()));
        assertEquals(true, RedTippingAuto.isOnOurSide(FieldConfig.bLaunchPoint.getX()));
    }

    // ---- never cross to the Blue side ----

    @Test
    public void keepOnOurSide_pullsBackPointsPastTheLimit_andLeavesTheRestAlone() {
        List<Position> path = new ArrayList<>();
        path.add(new Position(500, 1000, 0));
        path.add(new Position(RedTippingAuto.OUR_SIDE_LIMIT_X, 2000, 0)); // exactly on the limit: fine
        path.add(new Position(2500, 3000, 0));                            // on the Blue side
        path.add(new Position(FieldConfig.MIDLINE_X + 1, 400, 0));        // one mm over the midline

        int moved = RedTippingAuto.keepOnOurSide(path);

        assertEquals(2, moved);
        assertEquals(500, path.get(0).getX(), 1e-9);
        assertEquals(RedTippingAuto.OUR_SIDE_LIMIT_X, path.get(2).getX(), 1e-9);
        assertEquals(3000, path.get(2).getY(), 1e-9); // y is untouched
        assertEquals(RedTippingAuto.OUR_SIDE_LIMIT_X, path.get(3).getX(), 1e-9);
        assertEquals(0, RedTippingAuto.keepOnOurSide(path)); // second pass: nothing left to move
    }

    @Test
    public void ourSideLimit_keepsTheWholeRobotBodyOnOurSide() {
        double bodyEdge = RedTippingAuto.OUR_SIDE_LIMIT_X
                + org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint.WIDTH_MM / 2;
        assertEquals(true, bodyEdge <= FieldConfig.MIDLINE_X);
        // The launch points are real places we drive to, so the limit must not push them.
        assertEquals(true, FieldConfig.aLaunchPoint.getX() <= RedTippingAuto.OUR_SIDE_LIMIT_X);
        assertEquals(true, FieldConfig.bLaunchPoint.getX() <= RedTippingAuto.OUR_SIDE_LIMIT_X);
    }

    @Test
    public void keepOnOurSide_emptyPath_movesNothing() {
        assertEquals(0, RedTippingAuto.keepOnOurSide(new ArrayList<Position>()));
    }

    @Test
    public void keepOnOurSide_keepsYAndHeading() {
        List<Position> path = new ArrayList<>();
        path.add(new Position(3000, 1234, 1.5));
        RedTippingAuto.keepOnOurSide(path);
        assertEquals(1234, path.get(0).getY(), 1e-9);
        assertEquals(1.5, path.get(0).getTheta(), 1e-9);
    }

    @Test
    public void keepOnOurSide_launchPointsAndStartAreNotMoved() {
        List<Position> path = new ArrayList<>();
        path.add(new Position(FieldConfig.aLaunchPoint.getX(), FieldConfig.aLaunchPoint.getY(), 0));
        path.add(new Position(FieldConfig.bLaunchPoint.getX(), FieldConfig.bLaunchPoint.getY(), 0));
        path.add(new Position(1500, 3658 - 600, 0));
        assertEquals(0, RedTippingAuto.keepOnOurSide(path));
    }
}
