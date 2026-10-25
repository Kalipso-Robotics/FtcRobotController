package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Smoke test: the real RedTippingAuto runs to the end on the simulated robot. */
public class RedTippingAutoSimTest {

    @Test
    public void redTippingAutoRunsToTheEnd() throws Exception {
        // Start pose = the one RedTippingAuto.initializeRobot gives Odometry: (171.5 + 1218, 3658 - 171.5, -90 deg).
        SimRunner.Result result = SimRunner.runInNewJvm(
                "org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto",
                171.5 + 1218, 3658 - 171.5, Math.toRadians(-90));

        double[] first = result.frames.get(0);
        double[] last = result.frames.get(result.frames.size() - 1);
        System.out.println("SIM frames=" + result.frames.size() + " virtualSecondsUsed=" + result.virtualSecondsUsed
                + " finishedInTime=" + result.finishedInTime + " error=" + result.error);
        System.out.printf("SIM start true=(%.0f, %.0f, %.1f deg)%n", first[1], first[2], Math.toDegrees(first[3]));
        System.out.printf("SIM end   true=(%.0f, %.0f, %.1f deg)  odometry=(%.0f, %.0f, %.1f deg)%n",
                last[1], last[2], Math.toDegrees(last[3]), last[4], last[5], Math.toDegrees(last[6]));
        double nextPrint = 0;
        for (double[] f : result.frames) {
            if (f[0] >= nextPrint) {
                System.out.printf("SIM t=%5.1fs true=(%5.0f, %5.0f, %6.1f deg)  odometry=(%5.0f, %5.0f, %6.1f deg)%n",
                        f[0], f[1], f[2], Math.toDegrees(f[3]), f[4], f[5], Math.toDegrees(f[6]));
                nextPrint += 2;
            }
        }
        for (String warning : result.warnings) {
            System.out.println("SIM WARNING " + warning);
        }
        for (String event : result.events) {
            System.out.println("SIM event t=" + event.replace("|", "s  "));
        }
        System.out.println("SIM match starts at " + result.matchStartSeconds + " s");
        if (result.error != null) {
            System.out.println(result.error);
        }

        assertNull("the auto threw: " + result.error, result.error);
        assertTrue("the auto did not finish", result.finishedInTime);
        assertTrue("match should last about 30 virtual seconds", Math.abs(result.virtualSecondsUsed - 30) < 1);
    }

    /** The same scenario twice must give the same frames: that is what makes a run trustworthy. */
    @Test
    public void sameRunTwiceGivesTheSameFrames() throws Exception {
        String opMode = "org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto";
        SimRunner.Result a = SimRunner.runInNewJvm(opMode, 171.5 + 1218, 3658 - 171.5, Math.toRadians(-90));
        SimRunner.Result b = SimRunner.runInNewJvm(opMode, 171.5 + 1218, 3658 - 171.5, Math.toRadians(-90));

        int common = Math.min(a.frames.size(), b.frames.size());
        double worstMm = 0;
        for (int i = 0; i < common; i++) {
            worstMm = Math.max(worstMm, Math.hypot(a.frames.get(i)[1] - b.frames.get(i)[1], a.frames.get(i)[2] - b.frames.get(i)[2]));
        }
        System.out.println("SIM determinism: frames " + a.frames.size() + " vs " + b.frames.size() + ", worst position gap " + worstMm + " mm");
        assertEquals(a.frames.size(), b.frames.size());
        assertEquals(0.0, worstMm, 1.0);
    }
}
