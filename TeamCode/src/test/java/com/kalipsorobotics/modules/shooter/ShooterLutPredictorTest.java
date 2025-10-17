package com.kalipsorobotics.modules.shooter;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

import com.kalipsorobotics.math.Point;

/**
 * Unit test for ShooterLutPredictor
 * This test runs locally without requiring an Android device or FTC controller.
 * Now uses the pre-generated ShooterLutData instead of parsing JSON.
 */
public class ShooterLutPredictorTest {

    private ShooterLutPredictor predictor;

    @Before
    public void setUp() throws Exception {
        // Use real ShooterLutPredictor with binary data file
        String resourcePath = "ShooterLUT.bin";
        java.io.InputStream is = getClass().getClassLoader().getResourceAsStream(resourcePath);
        if (is == null) {
            throw new RuntimeException("Could not find resource: " + resourcePath);
        }
        predictor = new ShooterLutPredictor(is);
        is.close();
    }

//    @Test
//    public void testPredictionAtKnownDataPoint() throws Exception {
//        // Test with a known data point from the JSON (x=200, y=200)
//        // Expected values: rps=38.811670634920645, hood=0.5807272817460332
//        ShooterLutPredictor.Prediction prediction = predictor.predict(200, 200);
//
//        // Should return exact values when hitting a known point
//        assertEquals(38.811670634920645, prediction.rps, 0.001);
//        assertEquals(0.5807272817460332, prediction.hood, 0.001);
//    }
//
//    @Test
//    public void testPredictionInterpolation() throws Exception {
//        // Test with a point between known values (x=225, y=225)
//        // Should interpolate between nearby points
//        ShooterLutPredictor.Prediction prediction = predictor.predict(225, 225);
//
//        // Verify prediction is in reasonable range
//        assertTrue("RPS should be positive", prediction.rps > 0);
//        assertTrue("Hood should be within bounds", prediction.hood >= 0.23 && prediction.hood <= 0.8);
//    }
//
//    @Test
//    public void testHoodClamping() throws Exception {
//        // The predictor should clamp hood values to [0.23, 0.8]
//        ShooterLutPredictor.Prediction prediction = predictor.predict(200, 400);
//
//        assertTrue("Hood should not be below minimum", prediction.hood >= 0.23);
//        assertTrue("Hood should not be above maximum", prediction.hood <= 0.8);
//    }
//
//    @Test
//    public void testPredictionConsistency() throws Exception {
//        // Same input should always give same output
//        ShooterLutPredictor.Prediction pred1 = predictor.predict(500, 500);
//        ShooterLutPredictor.Prediction pred2 = predictor.predict(500, 500);
//
//        assertEquals("RPS should be consistent", pred1.rps, pred2.rps, 0.0001);
//        assertEquals("Hood should be consistent", pred1.hood, pred2.hood, 0.0001);
//    }

    @Test
    public void testMultiplePositions() throws Exception {
        // Test various positions to ensure predictor works across the range
        double[][] testPoints = {
            {Shooter.mmToPixel(Point.distance(Shooter.RED_TARGET_FROM_NEAR.getX(), Shooter.RED_TARGET_FROM_NEAR.getY(), 0, 0)), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(Point.distance(Shooter.RED_TARGET_FROM_NEAR.getX()/2, Shooter.RED_TARGET_FROM_NEAR.getY(), 0, 0)), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(300), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(150), Shooter.GOAL_HEIGHT_PIXELS}
        };

        System.out.println("\n╔════════════════════════════════════════════════════════════════════════════════════════╗");
        System.out.println("║                           ShooterLutPredictor Test Results                             ║");
        System.out.println("╠════════════╦════════════╦═════════════╦═════════════╦═══════════════╦══════════════════╣");
        System.out.println("║  X Pixel   ║  Y Pixel   ║   X (mm)    ║   Y (mm)    ║      RPS      ║   Hood Position  ║");
        System.out.println("╠════════════╬════════════╬═════════════╬═════════════╬═══════════════╬══════════════════╣");

        for (double[] point : testPoints) {
            ShooterLutPredictor.Prediction prediction = predictor.predict(point[0], point[1]);

            System.out.printf("║ %10.1f ║ %10.1f ║ %11.1f ║ %11.1f ║ %13.2f ║ %16.4f ║%n",
                            point[0], point[1],
                            Shooter.pixelToMM(point[0]), Shooter.pixelToMM(point[1]),
                            prediction.rps, prediction.hood);

            assertNotNull("Prediction should not be null", prediction);
            assertTrue("RPS should be positive for point (" + point[0] + ", " + point[1] + ")",
                       prediction.rps > 0);
            assertTrue("Hood should be within bounds for point (" + point[0] + ", " + point[1] + ")",
                       prediction.hood >= 0.23 && prediction.hood <= 0.8);
        }

        System.out.println("╚════════════╩════════════╩═════════════╩═════════════╩═══════════════╩══════════════════╝\n");
    }
}
