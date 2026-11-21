package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** OctoTune's spin step must use the counts/mm it just measured, not the static seeds. */
public class OctoSpinGeometryTest {

    @Test
    public void spinGeometry_usesGivenCountsPerMm() {
        double[] seeds = OctoConfig.spinGeometry(4000, -4000, 2000, 360, 1);
        double[] measured = OctoConfig.spinGeometry(4000, -4000, 2000, 360, 1,
                OctoConfig.COUNTS_PER_MM_X * 2, OctoConfig.COUNTS_PER_MM_X2 * 2, OctoConfig.COUNTS_PER_MM_Y * 2);
        for (int i = 0; i < seeds.length; i++) {
            assertEquals("offset " + i + " must halve when counts/mm doubles", seeds[i] / 2, measured[i], 1e-9);
        }
    }
}
