package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Test for ShooterManualDataLookup with linear interpolation.
 */
public class ShooterInterpolationDataLookupTest {

    private static final double HOOD_OFFSET = 0.25; // Must match ShooterManualDataLookup.HOOD_OFFSET
    private ShooterInterpolationDataLookup lookup;

    @Before
    public void setUp() {
        // Data is now static, no need to load from file
        lookup = new ShooterInterpolationDataLookup();
    }

    @Test
    public void testDataLoaded() {
        assertTrue("Should have loaded data points", lookup.getDataPointCount() > 0);
        assertEquals("Should have 20 data points", 20, lookup.getDataPointCount());
        System.out.println("Loaded " + lookup.getDataPointCount() + " data points");
        System.out.println("Distance range: " + lookup.getMinDistance() + "mm to " + lookup.getMaxDistance() + "mm");
    }

    @Test
    public void testExactDataPoints() {
        // Test exact matches from the CSV (with HOOD_OFFSET applied)
        // First point: 540,34,0.1 -> Hood = 0.1 + 0.25 = 0.35
        IShooterPredictor.ShooterParams params1 = lookup.predict(540);
        assertEquals("RPS at 540mm should be 34", 34.0, params1.rps, 0.01);
        assertEquals("Hood at 540mm should be 0.1 + offset", 0.1 + HOOD_OFFSET, params1.hoodPosition, 0.01);

        // Middle point: 1955,41,0.55 -> Hood = 0.55 + 0.25 = 0.8
        IShooterPredictor.ShooterParams params2 = lookup.predict(1955);
        assertEquals("RPS at 1955mm should be 41", 41.0, params2.rps, 0.01);
        assertEquals("Hood at 1955mm should be 0.55 + offset", 0.55 + HOOD_OFFSET, params2.hoodPosition, 0.01);

        // Last point: 3547,52,0.55 -> Hood = 0.55 + 0.25 = 0.8
        IShooterPredictor.ShooterParams params3 = lookup.predict(3547);
        assertEquals("RPS at 3547mm should be 52", 52.0, params3.rps, 0.01);
        assertEquals("Hood at 3547mm should be 0.55 + offset", 0.55 + HOOD_OFFSET, params3.hoodPosition, 0.01);
    }

    @Test
    public void testLinearInterpolation() {
        // Test interpolation between two points
        // Between 540 (RPS=34, Hood=0.1) and 682 (RPS=34, Hood=0.31)
        // Distance = 540 + (682-540)/2 = 611
        IShooterPredictor.ShooterParams params = lookup.predict(611);

        // t = (611 - 540) / (682 - 540) = 71/142 = 0.5
        // RPS = 34 + 0.5 * (34 - 34) = 34
        // Hood = 0.1 + 0.5 * (0.31 - 0.1) = 0.205, then + 0.25 offset = 0.455
        assertEquals("RPS at 611mm should be 34", 34.0, params.rps, 0.01);
        assertEquals("Hood at 611mm should be ~0.205 + offset", 0.205 + HOOD_OFFSET, params.hoodPosition, 0.01);

        System.out.println("Interpolated at 611mm: " + params);
    }

    @Test
    public void testEdgeCases() {
        // Test below minimum distance (should return first point with offset)
        // First point: 540mm, RPS=34, Hood=0.1 -> 0.1 + 0.25 = 0.35
        IShooterPredictor.ShooterParams params1 = lookup.predict(100);
        assertEquals("RPS below min should be 34", 34.0, params1.rps, 0.01);
        assertEquals("Hood below min should be 0.1 + offset", 0.1 + HOOD_OFFSET, params1.hoodPosition, 0.01);

        // Test above maximum distance (should return last point with offset)
        // Last point: 3547mm, RPS=52, Hood=0.55 -> 0.55 + 0.25 = 0.8
        IShooterPredictor.ShooterParams params2 = lookup.predict(5000);
        assertEquals("RPS above max should be 52", 52.0, params2.rps, 0.01);
        assertEquals("Hood above max should be 0.55 + offset", 0.55 + HOOD_OFFSET, params2.hoodPosition, 0.01);

        System.out.println("Below min (100mm): " + params1);
        System.out.println("Above max (5000mm): " + params2);
    }

    @Test
    public void testMultipleDistances() {
        // Test a range of distances to demonstrate the lookup
        double[] testDistances = {540, 700, 1000, 1500, 2000, 2500, 3000, 3547};

        System.out.println("\n╔════════════════════════════════════════════════════════════╗");
        System.out.println("║        Shooter Manual Data Lookup - Test Results          ║");
        System.out.println("║                  (with 0.25 hood offset)                   ║");
        System.out.println("╠══════════════╦═════════════╦══════════════════════════════╣");
        System.out.println("║ Distance (mm)║     RPS     ║      Hood Position           ║");
        System.out.println("╠══════════════╬═════════════╬══════════════════════════════╣");

        for (double distance : testDistances) {
            IShooterPredictor.ShooterParams params = lookup.predict(distance);
            System.out.printf("║ %12.0f ║ %11.2f ║ %28.4f ║%n",
                    distance, params.rps, params.hoodPosition);

            // Verify results are reasonable (with offset applied)
            assertTrue("RPS should be positive", params.rps > 0);
            assertTrue("Hood should be in valid range with offset",
                    params.hoodPosition >= (0.1 + HOOD_OFFSET) && params.hoodPosition <= (0.55 + HOOD_OFFSET));
        }

        System.out.println("╚══════════════╩═════════════╩══════════════════════════════╝\n");
    }

    @Test
    public void testInterpolationAccuracy() {
        // Test that interpolation is truly linear
        // Between 1580 (RPS=38, Hood=0.55) and 1698 (RPS=39, Hood=0.55)

        double d1 = 1580, rps1 = 38, hood1 = 0.55;
        double d2 = 1698, rps2 = 39, hood2 = 0.55;

        // Test at 25%, 50%, 75% between the points
        double[] percentages = {0.25, 0.5, 0.75};

        for (double pct : percentages) {
            double testDistance = d1 + pct * (d2 - d1);
            double expectedRPS = rps1 + pct * (rps2 - rps1);
            double expectedHood = hood1 + pct * (hood2 - hood1) + HOOD_OFFSET; // Add offset

            IShooterPredictor.ShooterParams params = lookup.predict(testDistance);

            assertEquals("RPS should match linear interpolation at " + (pct*100) + "%",
                    expectedRPS, params.rps, 0.001);
            assertEquals("Hood should match linear interpolation at " + (pct*100) + "% (with offset)",
                    expectedHood, params.hoodPosition, 0.001);

            System.out.printf("At %.0f%% between points (%.0fmm): RPS=%.2f, Hood=%.4f%n",
                    pct * 100, testDistance, params.rps, params.hoodPosition);
        }
    }
}
