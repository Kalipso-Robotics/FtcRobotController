package com.kalipsorobotics.modules.shooter;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Standalone test for ShooterLutPredictor that can run without JUnit
 * Usage: java StandaloneShooterLutPredictorTest
 */
public class StandaloneShooterLutPredictorTest {

    private static class Prediction {
        public final double rps;
        public final double hood;
        public Prediction(double rps, double hood) { this.rps = rps; this.hood = hood; }
    }

    private static class Row {
        final double x, y, rps, hood;
        Row(double x, double y, double rps, double hood) {
            this.x = x; this.y = y; this.rps = rps; this.hood = hood;
        }
    }

    private static final double MIN_HOOD_POS = 0.23;
    private static final double MAX_HOOD_POS = 0.8;
    private final List<Row> rows = new ArrayList<>();

    public StandaloneShooterLutPredictorTest(String jsonFilePath) throws Exception {
        // Simple JSON parsing without org.json dependency
        StringBuilder sb = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new FileReader(jsonFilePath))) {
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

        System.out.println("Loaded " + rows.size() + " data points from LUT");
    }

    private static double extractDouble(String json, String key) {
        Pattern pattern = Pattern.compile("\"" + key + "\"\\s*:\\s*([0-9.\\-]+)");
        Matcher matcher = pattern.matcher(json);
        if (matcher.find()) {
            return Double.parseDouble(matcher.group(1));
        }
        throw new RuntimeException("Key not found: " + key);
    }

    public Prediction predict(double xPixel, double yPixel) {
        final int K = 8;
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> -a[0]));

        for (Row r : rows) {
            double dx = r.x - xPixel, dy = r.y - yPixel;
            double d = Math.hypot(dx, dy);

            if (d == 0.0) {
                double clampedHood = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, r.hood));
                return new Prediction(r.rps, clampedHood);
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
        double clampedHood = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, predictedHood));

        return new Prediction(predictedRps, clampedHood);
    }

    public static void main(String[] args) {
        try {
            String dataFile = "src/test/resources/ShooterLUT.json";
            StandaloneShooterLutPredictorTest predictor = new StandaloneShooterLutPredictorTest(dataFile);

            System.out.println("\n=== Running Shooter LUT Predictor Tests ===\n");

            // Test 1: Known data point
            System.out.println("Test 1: Prediction at known data point (200, 200)");
            Prediction p1 = predictor.predict(200, 200);
            System.out.printf("  Expected: rps=38.812, hood=0.581%n");
            System.out.printf("  Actual:   rps=%.3f, hood=%.3f%n", p1.rps, p1.hood);
            assertTrue("RPS matches", Math.abs(p1.rps - 38.811670634920645) < 0.001);
            assertTrue("Hood matches", Math.abs(p1.hood - 0.5807272817460332) < 0.001);
            System.out.println("  ✓ PASS\n");

            // Test 2: Interpolation
            System.out.println("Test 2: Prediction with interpolation (225, 225)");
            Prediction p2 = predictor.predict(225, 225);
            System.out.printf("  Result: rps=%.3f, hood=%.3f%n", p2.rps, p2.hood);
            assertTrue("RPS is positive", p2.rps > 0);
            assertTrue("Hood in bounds", p2.hood >= 0.23 && p2.hood <= 0.8);
            System.out.println("  ✓ PASS\n");

            // Test 3: Hood clamping
            System.out.println("Test 3: Hood clamping");
            Prediction p3 = predictor.predict(200, 400);
            System.out.printf("  Result: hood=%.3f%n", p3.hood);
            assertTrue("Hood >= min", p3.hood >= 0.23);
            assertTrue("Hood <= max", p3.hood <= 0.8);
            System.out.println("  ✓ PASS\n");

            // Test 4: Consistency
            System.out.println("Test 4: Prediction consistency (500, 500)");
            Prediction p4a = predictor.predict(500, 500);
            Prediction p4b = predictor.predict(500, 500);
            System.out.printf("  First:  rps=%.6f, hood=%.6f%n", p4a.rps, p4a.hood);
            System.out.printf("  Second: rps=%.6f, hood=%.6f%n", p4b.rps, p4b.hood);
            assertTrue("RPS consistent", Math.abs(p4a.rps - p4b.rps) < 0.0001);
            assertTrue("Hood consistent", Math.abs(p4a.hood - p4b.hood) < 0.0001);
            System.out.println("  ✓ PASS\n");

            // Test 5: Multiple positions
            System.out.println("Test 5: Multiple positions");
            double[][] testPoints = {
                {300, 300},
                {400, 600},
                {700, 700},
                {850, 1000}
            };

            for (double[] point : testPoints) {
                Prediction p = predictor.predict(point[0], point[1]);
                System.out.printf("  (%.0f, %.0f) -> rps=%.2f, hood=%.3f%n",
                                  point[0], point[1], p.rps, p.hood);
                assertTrue("RPS positive", p.rps > 0);
                assertTrue("Hood in bounds", p.hood >= 0.23 && p.hood <= 0.8);
            }
            System.out.println("  ✓ PASS\n");

            System.out.println("=== All Tests Passed! ===");

        } catch (Exception e) {
            System.err.println("Test failed with error: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static void assertTrue(String message, boolean condition) {
        if (!condition) {
            throw new AssertionError("Assertion failed: " + message);
        }
    }
}
