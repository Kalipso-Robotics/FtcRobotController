package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.AutoDecider.HiveState;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Move;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.PartnerMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Pile;
import org.junit.Test;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Set;

/**
 * Tests the NEW decide() (it lives in DecideSimulation for now) and plays whole pretend matches
 * with it, on the laptop. Every pretend match is printed. (Android Studio: click the green arrow
 * in the left margin next to the class. Terminal: add "-i" to the gradle command.)
 */
public class DecideSimulationTest {
    private static final Set<Pile> ALL_PILES = EnumSet.allOf(Pile.class);
    private static final Set<Pile> NO_PILES = EnumSet.noneOf(Pile.class);

    /** Asks decide() with plenty of time: every job fits. Cell A up, Hive still, not at a launch point. */
    private static Move ask(PartnerMode mode, int ballsHeld, Cell upCell, Set<Pile> piles, int looseBallsSeen) {
        return AutoDecider.decide(mode, 25, 1, 5, 10, ballsHeld, false, upCell, false, 3, 0, piles,
                looseBallsSeen);
    }

    // ---- the rules, one at a time ----

    @Test
    public void holdingBalls_shootsTheUpCell_forEveryPartner() {
        for (PartnerMode mode : PartnerMode.values()) {
            assertEquals(Move.SHOOT_A, ask(mode, 4, Cell.A, ALL_PILES, 0));
            assertEquals(Move.SHOOT_B, ask(mode, 1, Cell.B, ALL_PILES, 0));
        }
    }

    @Test
    public void empty_goesToThePilesInOrder() {
        Set<Pile> piles = EnumSet.allOf(Pile.class);
        assertEquals(Move.GET_GARDEN, ask(PartnerMode.NOT_SOLO, 0, Cell.A, piles, 5));
        piles.remove(Pile.GARDEN);
        assertEquals(Move.GET_FLOWER_A, ask(PartnerMode.NOT_SOLO, 0, Cell.A, piles, 5));
        assertEquals(Move.GET_FLOWER_B, ask(PartnerMode.SOLO_NOTHING, 0, Cell.A, piles, 0));
    }

    @Test
    public void empty_noPilesLeft_looseBallsOrPark() {
        assertEquals(Move.GET_SEEN_BALLS, ask(PartnerMode.NOT_SOLO, 0, Cell.A, NO_PILES, 2));
        assertEquals(Move.PARK, ask(PartnerMode.NOT_SOLO, 0, Cell.A, NO_PILES, 0));
    }

    @Test
    public void hiveTipping_holdingBallsWaits_butEmptyJustGoesForBalls() {
        assertEquals(Move.WAIT, AutoDecider.decide(PartnerMode.NOT_SOLO, 25, 1, 5, 10, 4, false, Cell.A,
                true, 3, 0, ALL_PILES, 0));
        assertEquals(Move.GET_GARDEN, AutoDecider.decide(PartnerMode.NOT_SOLO, 25, 1, 5, 10, 0, false,
                Cell.A, true, 3, 0, ALL_PILES, 0));
    }

    // ---- time: the park depends on distance, and every job has to fit ----

    @Test
    public void farFromTheLoadingZone_parksEarly() {
        // 3.9 s left but the drive to the Loading Zone takes 3.5 s.
        assertEquals(Move.PARK, AutoDecider.decide(PartnerMode.NOT_SOLO, 3.9, 3.5, 6, 10, 4, false,
                Cell.A, false, 3, 0, ALL_PILES, 0));
    }

    @Test
    public void closeToTheLoadingZone_stillHasTimeForAQuickShot() {
        assertEquals(Move.SHOOT_A, AutoDecider.decide(PartnerMode.NOT_SOLO, 3.9, 0.4, 2, 10, 4, false,
                Cell.A, false, 3, 0, ALL_PILES, 0));
    }

    @Test
    public void aShotOrTripThatWouldCostThePark_isNotTaken() {
        // shot: 5 s left, drive + shot + park from the launch point needs 6 s
        assertEquals(Move.PARK, AutoDecider.decide(PartnerMode.NOT_SOLO, 5, 2, 6, 10, 4, false,
                Cell.A, false, 3, 0, ALL_PILES, 0));
        // trip: 9 s left needs 10 s, but 11 s is enough
        assertEquals(Move.PARK, AutoDecider.decide(PartnerMode.NOT_SOLO, 9, 2, 5, 10, 0, false,
                Cell.A, false, 3, 0, ALL_PILES, 0));
        assertEquals(Move.GET_GARDEN, AutoDecider.decide(PartnerMode.NOT_SOLO, 11, 2, 5, 10, 0, false,
                Cell.A, false, 3, 0, ALL_PILES, 0));
    }

    @Test
    public void lastShot_onlyIfItIsSureToTip() {
        // at the launch point, Cell A has its 3 nectar, we hold 4: 7 tips it
        assertEquals(Move.SHOOT_A, AutoDecider.decide(PartnerMode.NOT_SOLO, 2, 2, 5, 10, 4, true,
                Cell.A, false, 3, 0, ALL_PILES, 0));
        // holding only 3: 6 does not tip it, so park
        assertEquals(Move.PARK, AutoDecider.decide(PartnerMode.NOT_SOLO, 2, 2, 5, 10, 3, true,
                Cell.A, false, 3, 0, ALL_PILES, 0));
        // sure tip but the Hive is moving: do not shoot
        assertEquals(Move.PARK, AutoDecider.decide(PartnerMode.NOT_SOLO, 2, 2, 5, 10, 4, true,
                Cell.A, true, 3, 0, ALL_PILES, 0));
    }

    // ---- the camera ----

    @Test
    public void camera_beatsOurGuess_andUnknownFallsBackToIt() {
        assertEquals(Cell.B, AutoDecider.resolveUpCell(HiveState.B_UP, Cell.A));
        assertEquals(Cell.A, AutoDecider.resolveUpCell(HiveState.A_UP, Cell.B));
        assertEquals(Cell.B, AutoDecider.resolveUpCell(HiveState.UNKNOWN, Cell.B));
        assertEquals(Cell.A, AutoDecider.resolveUpCell(HiveState.TIPPING, Cell.A));
    }

    @Test
    public void notSolo_flowerBIsStillWorthAVisit_evenWithNoLooseBalls() {
        // An empty pile is found and skipped for free, so we do not skip Flower B.
        assertEquals(Move.GET_FLOWER_B, ask(PartnerMode.NOT_SOLO, 0, Cell.A, EnumSet.of(Pile.FLOWER_B), 0));
    }

    // ---- whole pretend matches (the edge cases) ----

    /** Plays one whole match, prints it, and returns every line it wrote. */
    private String playMatch(int scenario, boolean cameraIsOn) {
        DecideSimulation simulation = new DecideSimulation();
        simulation.currentReport = new ArrayList<>();
        simulation.playMatch(scenario, cameraIsOn);

        StringBuilder text = new StringBuilder();
        for (String line : simulation.currentReport) {
            text.append(line).append('\n');
        }
        System.out.println(text);
        return text.toString();
    }

    private static int count(String text, String piece) {
        int count = 0;
        for (int i = text.indexOf(piece); i >= 0; i = text.indexOf(piece, i + 1)) {
            count++;
        }
        return count;
    }

    @Test
    public void everyScenario_playsToTheEnd_withAPathForEveryDrive() {
        for (int scenario = 0; scenario < DecideSimulation.SCENARIO_COUNT; scenario++) {
            for (boolean camera : new boolean[]{true, false}) {
                String report = playMatch(scenario, camera);
                String which = DecideSimulation.scenarioName(scenario) + " camera=" + camera;
                assertTrue(which + ": the match never ended", report.contains("===== FINAL"));
                assertFalse(which + ": a drive had no path", report.contains("NO PATH"));
                assertFalse(which + ": ran out of time in the middle of a move", report.contains("NOT ENOUGH TIME"));
            }
        }
    }

    @Test
    public void everyScenario_withTheCamera_parks() {
        for (int scenario = 0; scenario < DecideSimulation.SCENARIO_COUNT; scenario++) {
            assertTrue(DecideSimulation.scenarioName(scenario) + ": did not park",
                    playMatch(scenario, true).contains("parked, +5 PARK"));
        }
    }

    @Test
    public void scenario1_partnerTipsWhileWeDrive_cameraCancelsTheShot_noCameraWastesIt() {
        assertTrue(playMatch(1, true).contains("shot CANCELLED"));
        assertTrue(playMatch(1, false).contains("balls WASTED"));
    }

    @Test
    public void scenario2_partnerShootsIntoOurCellFirst_cameraNoticesBeforeShooting() {
        assertTrue(playMatch(2, true).contains("shot CANCELLED"));
        assertTrue(playMatch(2, false).contains("balls WASTED"));
    }

    @Test
    public void scenario3_partnerTakesTheGarden_weFindItEmptyAndMoveOn() {
        String report = playMatch(3, true);
        assertEquals(1, count(report, "the pile was EMPTY"));
        assertTrue(report.contains("GET_FLOWER_A"));
    }

    @Test
    public void scenario4_partnerTakesTwoPiles_weFindBothEmpty() {
        assertEquals(2, count(playMatch(4, true), "the pile was EMPTY"));
    }

    /** The AUTO points on the FINAL line of a report. */
    private static int finalPoints(String report) {
        int start = report.indexOf("FINAL: ") + "FINAL: ".length();
        return Integer.parseInt(report.substring(start, report.indexOf(" AUTO points", start)));
    }

    @Test
    public void scenario5_aMiss_cameraKnowsTheCellDidNotTip_noCameraThinksItDidAndLosesPoints() {
        int withCamera = finalPoints(playMatch(5, true));
        int withoutCamera = finalPoints(playMatch(5, false));
        assertTrue("camera " + withCamera + " vs no camera " + withoutCamera, withCamera > withoutCamera);
    }

    @Test
    public void scenario6_partnerTipsTwiceInARow_weWaitMoreThanOnce() {
        assertTrue(count(playMatch(6, true), "DECISION: WAIT") >= 2);
    }

    @Test
    public void scenario7_partialLoad_weStillShootIt() {
        String report = playMatch(7, true);
        assertTrue(report.contains("picked up 2 balls"));
        assertTrue(report.contains("shot 2 into Cell"));
    }

    @Test
    public void scenario8_looseBalls_weGetOursAndNeverGoToTheOpponentsSide() {
        String report = playMatch(8, true);
        assertTrue(report.contains("DECISION: GET_SEEN_BALLS -> go to (800, 1800)"));
        assertFalse(report.contains("(2800, 1800)"));
        assertTrue(report.contains("looseBallsSeen=1"));
    }

    @Test
    public void scenario9_cameraLostForAWhile_weFallBackToCountingThenRecover() {
        String report = playMatch(9, true);
        assertTrue(report.contains("camera LOST"));
        assertTrue(report.contains("hive=A_UP") || report.contains("hive=B_UP"));
    }
}
