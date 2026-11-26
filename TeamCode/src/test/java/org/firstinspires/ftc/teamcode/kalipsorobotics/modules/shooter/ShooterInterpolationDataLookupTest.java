package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Pins down ShooterInterpolationDataLookup: safe on an empty or one-row table, exact rows,
 * linear interpolation between rows, clamping to the first/last row outside the table,
 * and no duplicated rows when the lookup is constructed more than once.
 */
public class ShooterInterpolationDataLookupTest {
    private static final double TOL = 1e-9;
    // Hood values inside [MIN_HOOD, MAX_HOOD] so clampHood does not change them.
    private static final double HOOD_A = 0.15;
    private static final double HOOD_B = 0.20;
    private static final double HOOD_C = 0.25;

    private double[] savedData;
    private double savedRpsOffset;
    private double savedHoodOffset;

    @Before
    public void saveConfig() {
        savedData = ShooterInterpolationConfig.shooterData;
        savedRpsOffset = ShooterInterpolationConfig.rpsOffset;
        savedHoodOffset = ShooterInterpolationConfig.hoodOffset;
        ShooterInterpolationConfig.rpsOffset = 0;
        ShooterInterpolationConfig.hoodOffset = 0;
    }

    @After
    public void restoreConfig() {
        ShooterInterpolationConfig.shooterData = savedData;
        ShooterInterpolationConfig.rpsOffset = savedRpsOffset;
        ShooterInterpolationConfig.hoodOffset = savedHoodOffset;
    }

    private static ShooterInterpolationDataLookup lookupFor(double... rows) {
        ShooterInterpolationConfig.shooterData = rows;
        return new ShooterInterpolationDataLookup();
    }

    private static void assertParams(double rps, double hood, IShooterPredictor.ShooterParams p) {
        assertEquals("rps", rps, p.shooterRPS, TOL);
        assertEquals("hood", hood, p.hoodPosition, TOL);
    }

    @Test
    public void emptyTable_returnsSafeDefault() {
        ShooterInterpolationDataLookup lookup = lookupFor();
        assertEquals(0, lookup.getDataPointCount());
        for (double d : new double[]{0, 1500, 10000}) {
            assertParams(ShooterInterpolationConfig.DEFAULT_RPS, ShooterInterpolationConfig.MIN_HOOD, lookup.predict(d));
        }
    }

    @Test
    public void emptyTable_appliesDriverOffsets() {
        ShooterInterpolationConfig.rpsOffset = 2;
        ShooterInterpolationConfig.hoodOffset = 0.01;
        ShooterInterpolationDataLookup lookup = lookupFor();
        assertParams(ShooterInterpolationConfig.DEFAULT_RPS + 2,
                ShooterInterpolationConfig.MIN_HOOD + ShooterInterpolationConfig.HOOD_INIT_OFFSET + 0.01, lookup.predict(1500));
    }

    @Test
    public void oneRow_returnsThatRowForAnyDistance() {
        ShooterInterpolationDataLookup lookup = lookupFor(1500, 40, HOOD_A);
        for (double d : new double[]{0, 1500, 10000}) {
            assertParams(40, HOOD_A, lookup.predict(d));
        }
    }

    @Test
    public void exactRows_returnThoseRows() {
        ShooterInterpolationDataLookup lookup = lookupFor(1000, 40, HOOD_A, 2000, 50, HOOD_B, 3000, 60, HOOD_C);
        assertParams(40, HOOD_A, lookup.predict(1000));
        assertParams(50, HOOD_B, lookup.predict(2000));
        assertParams(60, HOOD_C, lookup.predict(3000));
    }

    @Test
    public void twoRows_interpolateLinearly() {
        ShooterInterpolationDataLookup lookup = lookupFor(1000, 40, HOOD_A, 2000, 50, HOOD_B);
        assertParams(42.5, HOOD_A + 0.25 * (HOOD_B - HOOD_A), lookup.predict(1250));
        assertParams(45, (HOOD_A + HOOD_B) / 2, lookup.predict(1500));
    }

    @Test
    public void threeRows_interpolateWithinTheRightSegment() {
        ShooterInterpolationDataLookup lookup = lookupFor(1000, 40, HOOD_A, 2000, 50, HOOD_B, 3000, 60, HOOD_C);
        assertParams(45, (HOOD_A + HOOD_B) / 2, lookup.predict(1500));
        assertParams(57.5, HOOD_B + 0.75 * (HOOD_C - HOOD_B), lookup.predict(2750));
    }

    @Test
    public void aboveMaximum_returnsLastRow() {
        ShooterInterpolationDataLookup lookup = lookupFor(1000, 40, HOOD_A, 2000, 50, HOOD_B, 3000, 60, HOOD_C);
        assertParams(60, HOOD_C, lookup.predict(5000));
    }

    @Test
    public void belowMinimum_returnsFirstRow() {
        ShooterInterpolationDataLookup lookup = lookupFor(1000, 40, HOOD_A, 2000, 50, HOOD_B, 3000, 60, HOOD_C);
        assertParams(40, HOOD_A, lookup.predict(100));
    }

    /** The real tuned table: whole rows, sorted by distance, hood inside the LUT range, RPS under the clamp. */
    @Test
    public void realTable_isWellFormed() {
        double[] rows = savedData;
        assertEquals("rows of 3", 0, rows.length % 3);
        for (int i = 0; i < rows.length; i += 3) {
            if (i > 0) org.junit.Assert.assertTrue("sorted at row " + i / 3, rows[i] > rows[i - 3]);
            org.junit.Assert.assertTrue("rps row " + i / 3, rows[i + 1] > 0 && rows[i + 1] <= ShooterInterpolationConfig.MAX_RPS);
            org.junit.Assert.assertTrue("hood row " + i / 3, rows[i + 2] >= ShooterInterpolationConfig.MIN_HOOD
                    && rows[i + 2] <= ShooterInterpolationConfig.MAX_HOOD);
        }
        ShooterInterpolationDataLookup lookup = lookupFor(rows);
        assertEquals(rows.length / 3, lookup.getDataPointCount());
        assertParams(rows[1], rows[2], lookup.predict(rows[0])); // exact first row comes back unchanged
    }

    @Test
    public void constructingTwice_doesNotDuplicateRows() {
        lookupFor(1000, 40, HOOD_A, 2000, 50, HOOD_B);
        ShooterInterpolationDataLookup second = new ShooterInterpolationDataLookup();
        assertEquals(2, second.getDataPointCount());
    }
}
