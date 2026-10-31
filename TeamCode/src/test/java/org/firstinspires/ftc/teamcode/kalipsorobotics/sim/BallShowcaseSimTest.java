package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** The sim-only showcase auto runs the real shooter, intake, stopper and turret code against the ball world. */
public class BallShowcaseSimTest {

    private static final String SHOWCASE = "org.firstinspires.ftc.teamcode.kalipsorobotics.simdemo.BallShowcaseAuto";
    private static final double START_X = 1389.5, START_Y = 3657.6 - 229 - 3;

    private static long count(SimRunner.Result result, String contains) {
        return result.events.stream().filter(e -> e.contains(contains)).count();
    }

    @Test
    public void theShowcaseShootsPicksUpAndTipsTheHive() throws Exception {
        SimRunner.Result result = SimRunner.runInNewJvm(SHOWCASE, START_X, START_Y, Math.toRadians(-90));
        assertNull("the auto threw: " + result.error, result.error);
        assertTrue(result.finishedInTime);
        assertTrue("no setup warnings with a legal start: " + result.warnings, result.warnings.isEmpty());

        long shots = count(result, "|shot a ");
        long inCell = count(result, "went into");
        long pickups = count(result, "|picked up");
        long tips = count(result, " tipped");
        System.out.printf("SIM showcase: shots=%d inCell=%d pickups=%d tips=%d assumed=%s%n", shots, inCell, pickups, tips, result.assumed);
        assertTrue("12 shots (4 preload, 4 Garden, 4 Flower B), got " + shots, shots >= 12);
        assertTrue("most shots land in a Cell, got " + inCell + " of " + shots, inCell >= shots - 1);
        assertTrue("the Garden and Flower B are picked up, got " + pickups, pickups >= 8);
        assertTrue("the preload plus the Cell's 3 nectar tips the Hive, and the later shots tip it again, got " + tips, tips >= 1);
        assertEquals("every ball the code asked the hardware for was simulated", 0, result.assumed.size());
        assertEquals(34, result.ballInfo.size()); // 16 Flower + 8 Garden + 6 Cell + 4 preload
    }

    @Test
    public void theShowcaseRepeatsExactly() throws Exception {
        SimRunner.Result a = SimRunner.runInNewJvm(SHOWCASE, START_X, START_Y, Math.toRadians(-90));
        SimRunner.Result b = SimRunner.runInNewJvm(SHOWCASE, START_X, START_Y, Math.toRadians(-90));
        assertEquals(a.events, b.events);
        assertEquals(a.ballMoves.size(), b.ballMoves.size());
        double[] last = a.frames.get(a.frames.size() - 1), lastB = b.frames.get(b.frames.size() - 1);
        assertEquals(last[1], lastB[1], 0.5);
        assertEquals(last[2], lastB[2], 0.5);
    }
}
