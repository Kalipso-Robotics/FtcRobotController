package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * With nothing in the way, the Rapier world and the plain drivetrain model must agree. They are two
 * separate implementations of the same wheel model, so a gap means a mistake in one of them.
 * Both get the same commands in an open part of the field.
 */
public class WorldParityTest {

    private static final String[] MOTORS = {"fLeft", "fRight", "bLeft", "bRight"};

    /** Wheel powers {fLeft, fRight, bLeft, bRight} held for a number of seconds. */
    private static final double[][] SCRIPT = {
            {0.2, 0.2, 0.2, 0.2, 0.6},      // forward
            {0.2, -0.2, -0.2, 0.2, 0.6},    // sideways (the code's mix: FL and BR positive, FR and BL negative)
            {0.2, -0.2, 0.2, -0.2, 0.6},    // turn on the spot
            {0.15, 0.3, 0.3, 0.15, 0.5},    // a mix of forward and sideways
            {0, 0, 0, 0, 0.5},              // stop (brake)
    };

    @Test
    public void rapierAgreesWithTheOpenFieldModel() throws Exception {
        try (RapierWorld rapier = new RapierWorld(); KinematicWorld open = new KinematicWorld()) {
            rapier.setPose(500, 600, 0); // clear of the walls, the Hive legs and Flower B, even while spinning
            open.setPose(500, 600, 0);

            double worstMm = 0, worstRad = 0, worstTicks = 0;
            double firstGapAt = -1;
            long now = 0;
            for (double[] segment : SCRIPT) {
                for (int i = 0; i < 4; i++) {
                    rapier.setMotorPower(MOTORS[i], segment[i]);
                    open.setMotorPower(MOTORS[i], segment[i]);
                }
                long end = now + Math.round(segment[4] * 1e9);
                for (; now < end; now += 7_300_000L) { // 7.3 ms: not a multiple of the 1 ms step, on purpose
                    rapier.advanceTo(now);
                    open.advanceTo(now);
                    double gapMm = Math.hypot(rapier.getX() - open.getX(), rapier.getY() - open.getY());
                    if (gapMm > 2 && firstGapAt < 0) {
                        firstGapAt = now / 1e9;
                    }
                    worstMm = Math.max(worstMm, gapMm);
                    worstRad = Math.max(worstRad, Math.abs(rapier.getTheta() - open.getTheta()));
                    for (String pod : new String[]{"bLeft", "bRight", "fRight"}) {
                        worstTicks = Math.max(worstTicks, Math.abs(rapier.rawEncoderTicks(pod) - open.rawEncoderTicks(pod)));
                    }
                }
                now = end;
            }
            System.out.println("SIM parity: first gap over 2 mm at " + firstGapAt + " s; rapier events " + rapier.drainEvents());
            System.out.printf("SIM parity: worst gap %.3f mm, %.5f rad, %.2f ticks; end rapier=(%.1f, %.1f) open=(%.1f, %.1f)%n",
                    worstMm, worstRad, worstTicks, rapier.getX(), rapier.getY(), open.getX(), open.getY());
            assertTrue("position gap " + worstMm + " mm", worstMm < 2);
            assertTrue("heading gap " + worstRad + " rad", worstRad < 0.003);
            assertTrue("encoder gap " + worstTicks + " ticks", worstTicks < 40); // 40 ticks = 2 mm
        }
    }
}
