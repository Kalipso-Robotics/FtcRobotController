package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.util.ArrayList;
import java.util.Arrays;

/** The live-mode line protocol and the wall-clock pacing (SimLive). No robot code runs here. */
public class SimLiveTest {

    @Test
    public void gamepadLineSetsSticksTriggersAndButtons() {
        Gamepad one = new Gamepad(), two = new Gamepad();
        // buttons: a (bit 0) + right_bumper (bit 5) + dpad_right (bit 9)
        assertTrue(SimLive.applyLine("gp 1 0.5 -0.25 1 0 0.75 1 " + (1 + 32 + 512), one, two));
        assertEquals(0.5f, one.left_stick_x, 1e-6);
        assertEquals(-0.25f, one.left_stick_y, 1e-6);
        assertEquals(1f, one.right_stick_x, 1e-6);
        assertEquals(0.75f, one.left_trigger, 1e-6);
        assertEquals(1f, one.right_trigger, 1e-6);
        assertTrue(one.a && one.right_bumper && one.dpad_right);
        assertFalse(one.b || one.x || one.y || one.left_bumper || one.dpad_up || one.start);
        assertEquals(0f, two.left_stick_x, 0);   // gamepad 2 untouched
    }

    @Test
    public void gamepadTwoAndOutOfRangeValuesAreClamped() {
        Gamepad one = new Gamepad(), two = new Gamepad();
        assertTrue(SimLive.applyLine("gp 2 5 -5 0 0 -1 9 0", one, two));
        assertEquals(1f, two.left_stick_x, 0);
        assertEquals(-1f, two.left_stick_y, 0);
        assertEquals(0f, two.left_trigger, 0);
        assertEquals(1f, two.right_trigger, 0);
        assertFalse(two.a);
    }

    @Test
    public void malformedLinesAreIgnored() {
        Gamepad one = new Gamepad(), two = new Gamepad();
        for (String bad : new String[]{"", "hello", "gp 3 0 0 0 0 0 0 0", "gp 1 0 0 0 0 0 0", "gp 1 x 0 0 0 0 0 0", "gp 1 NaN 0 0 0 0 0 0", "gp 1 1 0 0 0 0 0 abc"}) {
            assertFalse(bad, SimLive.applyLine(bad, one, two));
        }
        assertEquals(0f, one.left_stick_x, 0);
    }

    @Test
    public void stopLineAndEndOfInputBothEndTheSession() throws Exception {
        for (String input : new String[]{"gp 1 0 0 0 0 0 0 0\nstop\n", "gp 1 0 0 0 0 0 0 0\n"}) {
            SimLive live = new SimLive();
            live.startReader(new ByteArrayInputStream(input.getBytes()), new Gamepad(), new Gamepad());
            long deadline = System.currentTimeMillis() + 2000;
            while (!live.isStopRequested() && System.currentTimeMillis() < deadline) {
                Thread.sleep(5);
            }
            assertTrue(live.isStopRequested());
        }
    }

    @Test
    public void paceWaitsForTheWallClockOnlyAfterPacingStarts() {
        SimLive live = new SimLive();
        long t0 = System.nanoTime();
        live.pace(5_000_000_000L);   // not started: returns at once
        assertTrue(System.nanoTime() - t0 < 200_000_000L);

        live.startPacing(1_000_000_000L);
        t0 = System.nanoTime();
        live.pace(1_300_000_000L);   // 0.3 virtual seconds on
        long waited = System.nanoTime() - t0;
        assertTrue("waited " + waited, waited >= 290_000_000L && waited < 600_000_000L);
        t0 = System.nanoTime();
        live.pace(1_100_000_000L);   // already in the past
        assertTrue(System.nanoTime() - t0 < 50_000_000L);
    }

    @Test
    public void frameLineIsJsonWithNullsForUnknownOdometry() {
        String line = SimLive.frameLine(1.5, new double[]{100, 200, 0.5}, new double[]{Double.NaN, Double.NaN, Double.NaN}, new double[]{10, 0.5, 30}, 2,
                new double[]{0, 0.1}, new ArrayList<>(Arrays.asList(new double[]{3, 10, 20, 30, 0})), null);
        assertEquals("#frame {\"t\":1.5000,\"x\":100.0000,\"y\":200.0000,\"th\":0.5000,\"ox\":null,\"oy\":null,\"oth\":null,\"turretDeg\":10.0000,\"hood\":0.5000,\"rps\":30.0000,"
                + "\"held\":2,\"hive\":[0.0000,0.1000],\"balls\":[[3,10.0000,20.0000,30.0000]]}", line);
    }

    @Test
    public void setChangesADashboardConfigAndRefusesAnythingElse() throws Exception {
        double old = org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig.RELEASE_LATENCY_MS;
        try {
            assertEquals("SOTMConfig.RELEASE_LATENCY_MS 75.0", SimLive.setConfig("SOTMConfig.RELEASE_LATENCY_MS", "75"));
            assertEquals(75, org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig.RELEASE_LATENCY_MS, 0);
        } finally {
            org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig.RELEASE_LATENCY_MS = old;
        }
        for (String bad : new String[]{"ModuleConfig.STOPPER_SERVO_OPEN_POS", "java.lang.System.out", "SOTMConfig.GOALS", "NoSuch.X"}) {
            try {
                SimLive.setConfig(bad, "1");
                throw new AssertionError(bad + " should be refused");
            } catch (ReflectiveOperationException | IllegalArgumentException expected) {
                // final, outside decode.configs, not a number, or missing
            }
        }
    }
}
