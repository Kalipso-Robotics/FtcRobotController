package com.kalipsorobotics.modules.shooter;

import org.junit.Before;
import org.junit.Test;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.*;

import com.kalipsorobotics.math.Point;

/**
 * Unit test for ShooterLutPredictor
 * This test runs locally without requiring an Android device or FTC controller.
 */
public class ShooterLutPredictorTest {

    private TestShooterLutPredictor predictor;

    /**
     * Test implementation of ShooterLutPredictor that reads from file system
     * instead of Android assets
     */
    private static class TestShooterLutPredictor {
        private static final double MIN_HOOD_POS = 0.23;
        private static final double MAX_HOOD_POS = 0.8;

        private static class Row {
            final double x, y, rps, hood;
            Row(double x, double y, double rps, double hood) {
                this.x = x;
                this.y = y;
                this.rps = rps;
                this.hood = hood;
            }
        }

        private final List<Row> rows = new ArrayList<>();

        public TestShooterLutPredictor(String jsonFilePath) throws IOException {
            // Simple JSON parsing without org.json dependency (which isn't available in unit tests)
            StringBuilder sb = new StringBuilder();
            try (BufferedReader reader = new BufferedReader(new java.io.FileReader(jsonFilePath))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    sb.append(line);
                }
            }

            String json = sb.toString();

            // Extract data array using regex
            Pattern dataPattern = Pattern.compile("\"data\"\\s*:\\s*\\[(.*?)\\](?=\\s*})");
            Matcher dataMatcher = dataPattern.matcher(json);

            if (dataMatcher.find()) {
                String dataArray = dataMatcher.group(1);

                // Extract individual objects
                Pattern objPattern = Pattern.compile("\\{([^}]+)\\}");
                Matcher objMatcher = objPattern.matcher(dataArray);

                while (objMatcher.find()) {
                    String obj = objMatcher.group(1);

                    double x = extractDouble(obj, "x_pixel");
                    double y = extractDouble(obj, "y_pixel");
                    double rps = extractDouble(obj, "rps");
                    double hood = extractDouble(obj, "hood");

                    rows.add(new Row(x, y, rps, hood));
                }
            }
        }

        private static double extractDouble(String json, String key) {
            Pattern pattern = Pattern.compile("\"" + key + "\"\\s*:\\s*([0-9.\\-]+)");
            Matcher matcher = pattern.matcher(json);
            if (matcher.find()) {
                return Double.parseDouble(matcher.group(1));
            }
            throw new RuntimeException("Key not found: " + key);
        }

        public ShooterLutPredictor.Prediction predict(double xPixel, double yPixel) {
            // KNN (K=8), inverse-distance weighting (Shepard). If exact hit, return it.
            final int K = 8;
            PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> -a[0])); // max-heap by dist

            for (Row r : rows) {
                double dx = r.x - xPixel, dy = r.y - yPixel;
                double d = Math.hypot(dx, dy);

                if (d == 0.0) {
                    double clampedHood = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, r.hood));
                    return new ShooterLutPredictor.Prediction(r.rps, clampedHood);
                }

                if (pq.size() < K) {
                    pq.offer(new double[]{d, r.rps, r.hood});
                } else if (d < pq.peek()[0]) {
                    pq.poll();
                    pq.offer(new double[]{d, r.rps, r.hood});
                }
            }

            double wsum = 0, rpsSum = 0, hoodSum = 0;
            while (!pq.isEmpty()) {
                double[] e = pq.poll();
                double w = 1.0 / (e[0] + 1e-6);
                wsum += w;
                rpsSum += w * e[1];
                hoodSum += w * e[2];
            }

            double predictedRps = rpsSum / wsum;
            double predictedHood = hoodSum / wsum;

            // Clamp hood position to valid bounds
            double clampedHood = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, predictedHood));

            return new ShooterLutPredictor.Prediction(predictedRps, clampedHood);
        }
    }

    @Before
    public void setUp() throws IOException {
        // Use direct path to assets folder
        String resourcePath = "src/main/assets/ShooterLUT.json";
        predictor = new TestShooterLutPredictor(resourcePath);
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
