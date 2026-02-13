package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

/**
 * Unit test for ShooterLutPredictor
 * This test runs locally without requiring an Android device or FTC controller.
 * Now uses the pre-generated ShooterLutData instead of parsing JSON.
 */
public class ShooterLutPredictorTest {

    private ShooterLutPredictorInternal predictorCapped;
    private ShooterLutPredictorInternal predictorLUT2;

    @Before
    public void setUp() throws Exception {
        // Load CappedShooterLUT.bin
        String assetsPath1 = "src/main/assets/CappedShooterLUT.bin";
        java.io.File file1 = new java.io.File(assetsPath1);
        if (!file1.exists()) {
            throw new RuntimeException("Could not find asset file: " + file1.getAbsolutePath());
        }
        java.io.InputStream is1 = new java.io.FileInputStream(file1);
        predictorCapped = new ShooterLutPredictorInternal(is1);
        is1.close();

        // Load ShooterLUT2.bin
        String assetsPath2 = "src/main/assets/ShooterLUT2.bin";
        java.io.File file2 = new java.io.File(assetsPath2);
        if (!file2.exists()) {
            throw new RuntimeException("Could not find asset file: " + file2.getAbsolutePath());
        }
        java.io.InputStream is2 = new java.io.FileInputStream(file2);
        predictorLUT2 = new ShooterLutPredictorInternal(is2);
        is2.close();
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
            {Shooter.mmToPixel(Point.distance(Shooter.TARGET_POINT.getX(), Shooter.TARGET_POINT.getY(), 0, 0)), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(Point.distance(Shooter.TARGET_POINT.getX()/2, Shooter.TARGET_POINT.getY(), 0, 0)), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(2500), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(2800), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(3000), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(300), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(150), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(600), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(620), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(640), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(660), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(665), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(670), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(675), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(680), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(700), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(710), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(720), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(730), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(740), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(745), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(750), Shooter.GOAL_HEIGHT_PIXELS},
            {Shooter.mmToPixel(752), Shooter.GOAL_HEIGHT_PIXELS},
        };

        // Table 1: CappedShooterLUT.bin Results
        System.out.println("\n╔════════════════════════════════════════════════════════════════════════════════════════╗");
        System.out.println("║                    CappedShooterLUT.bin - Test Results                                 ║");
        System.out.println("╠════════════╦════════════╦═════════════╦═════════════╦═══════════════╦══════════════════╣");
        System.out.println("║  X Pixel   ║  Y Pixel   ║   X (mm)    ║   Y (mm)    ║      RPS      ║   Hood Position  ║");
        System.out.println("╠════════════╬════════════╬═════════════╬═════════════╬═══════════════╬══════════════════╣");

        for (double[] point : testPoints) {
            ShooterLutPredictorInternal.Prediction prediction = predictorCapped.predict(point[0], point[1]);

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

        // Table 2: ShooterLUT2.bin Results
        System.out.println("╔════════════════════════════════════════════════════════════════════════════════════════╗");
        System.out.println("║                      ShooterLUT2.bin - Test Results                                    ║");
        System.out.println("╠════════════╦════════════╦═════════════╦═════════════╦═══════════════╦══════════════════╣");
        System.out.println("║  X Pixel   ║  Y Pixel   ║   X (mm)    ║   Y (mm)    ║      RPS      ║   Hood Position  ║");
        System.out.println("╠════════════╬════════════╬═════════════╬═════════════╬═══════════════╬══════════════════╣");

        for (double[] point : testPoints) {
            ShooterLutPredictorInternal.Prediction prediction = predictorLUT2.predict(point[0], point[1]);

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

        // Table 3: Difference Between LUTs
        System.out.println("╔════════════════════════════════════════════════════════════════════════════════════════╗");
        System.out.println("║                         Difference (LUT2 - Capped)                                     ║");
        System.out.println("╠════════════╦════════════╦═════════════╦═════════════╦═══════════════╦══════════════════╣");
        System.out.println("║  X Pixel   ║  Y Pixel   ║   X (mm)    ║   Y (mm)    ║   Δ RPS       ║   Δ Hood         ║");
        System.out.println("╠════════════╬════════════╬═════════════╬═════════════╬═══════════════╬══════════════════╣");

        for (double[] point : testPoints) {
            ShooterLutPredictorInternal.Prediction predictionCapped = predictorCapped.predict(point[0], point[1]);
            ShooterLutPredictorInternal.Prediction predictionLUT2 = predictorLUT2.predict(point[0], point[1]);

            double deltaRPS = predictionLUT2.rps - predictionCapped.rps;
            double deltaHood = predictionLUT2.hood - predictionCapped.hood;

            System.out.printf("║ %10.1f ║ %10.1f ║ %11.1f ║ %11.1f ║ %+13.2f ║ %+16.4f ║%n",
                            point[0], point[1],
                            Shooter.pixelToMM(point[0]), Shooter.pixelToMM(point[1]),
                            deltaRPS, deltaHood);
        }

        System.out.println("╚════════════╩════════════╩═════════════╩═════════════╩═══════════════╩══════════════════╝\n");
    }
}
