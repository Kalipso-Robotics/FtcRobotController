package com.kalipsorobotics.modules.shooter;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;
import org.json.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;

public class ShooterLutPredictor {
    private static final double MIN_HOOD_POS = 0.23;
    private static final double MAX_HOOD_POS = 0.8;

    // Gradient smoothing constants
    private static final double STEEP_GRADIENT_THRESHOLD = 0.5; // RPS change per 10 pixels
    private static final int SMOOTHING_K = 12; // Use more neighbors when in steep gradient regions

    public static class Prediction {
        public final double rps;
        public final double hood;
        public Prediction(double rps, double hood) { this.rps = rps; this.hood = hood; }
    }
    private static class Row {
        final double x, y, rps, hood;
        Row(double x, double y, double rps, double hood) { this.x=x; this.y=y; this.rps=rps; this.hood=hood; }
    }
    private final List<Row> rows = new ArrayList<>();

    public ShooterLutPredictor(Context ctx, String assetName) throws IOException, JSONException {
        AssetManager am = ctx.getAssets();
        try (InputStream is = am.open(assetName)) {
            StringBuilder sb = new StringBuilder();
            byte[] buffer = new byte[1024];
            int bytesRead;
            while ((bytesRead = is.read(buffer)) != -1) {
                sb.append(new String(buffer, 0, bytesRead, StandardCharsets.UTF_8));
            }
            String json = sb.toString();
            JSONObject root = new JSONObject(json);
            JSONArray data = root.getJSONArray("data");
            for (int i=0; i<data.length(); i++) {
                JSONObject o = data.getJSONObject(i);
                rows.add(new Row(o.getDouble("x_pixel"), o.getDouble("y_pixel"),
                        o.getDouble("rps"), o.getDouble("hood")));
            }
        }
    }

    public Prediction predict(double xPixel, double yPixel) {
        // KNN with gradient-aware smoothing. If exact hit, return it.
        final int K = 8;
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> -a[0])); // max-heap by dist
        for (Row r : rows) {
            double dx = r.x - xPixel, dy = r.y - yPixel;
            double d = Math.hypot(dx, dy);
            if (d == 0.0) {
                // Check if exact hit is within bounds
                if (r.hood >= MIN_HOOD_POS && r.hood <= MAX_HOOD_POS) {
                    return new Prediction(r.rps, r.hood);
                }
            }
            if (pq.size() < K) pq.offer(new double[]{d, r.rps, r.hood, r.x, r.y});
            else if (d < pq.peek()[0]) { pq.poll(); pq.offer(new double[]{d, r.rps, r.hood, r.x, r.y}); }
        }

        // Extract K nearest neighbors
        List<double[]> neighbors = new ArrayList<>(pq);

        // Check if we're in a steep gradient region
        boolean steepGradient = detectSteepGradient(neighbors);

        // If steep gradient detected, use more neighbors with stronger distance weighting
        if (steepGradient) {
            Log.w("ShooterLutPredictor", "Steep gradient detected at (" + xPixel + ", " + yPixel +
                  ") - using enhanced smoothing");
            return predictWithSmoothing(xPixel, yPixel);
        }

        // Standard prediction
        double wsum = 0, rpsSum = 0, hoodSum = 0;
        for (double[] e : neighbors) {
            double w = 1.0 / (e[0] + 1e-6);
            wsum += w; rpsSum += w * e[1]; hoodSum += w * e[2];
        }
        double predictedRps = rpsSum / wsum;
        double predictedHood = hoodSum / wsum;

        Log.d("LUT_Standard", String.format(
            "Query: (%.1f, %.1f) | Standard prediction: RPS=%.2f, Hood=%.4f",
            xPixel, yPixel, predictedRps, predictedHood
        ));

        // If hood position is out of bounds, find next best combination
        if (predictedHood < MIN_HOOD_POS || predictedHood > MAX_HOOD_POS) {
            return findBestValidCombination(xPixel, yPixel);
        }

        return new Prediction(predictedRps, predictedHood);
    }

    /**
     * Detect if the K nearest neighbors have steep RPS gradients
     * @param neighbors list of nearest neighbors [distance, rps, hood, x, y]
     * @return true if steep gradient detected
     */
    private boolean detectSteepGradient(List<double[]> neighbors) {
        if (neighbors.size() < 2) return false;

        // Calculate max RPS gradient among neighbors
        double maxGradient = 0;
        double[] maxGradientPair = null;
        for (int i = 0; i < neighbors.size(); i++) {
            for (int j = i + 1; j < neighbors.size(); j++) {
                double[] n1 = neighbors.get(i);
                double[] n2 = neighbors.get(j);

                double rps1 = n1[1], rps2 = n2[1];
                double x1 = n1[3], y1 = n1[4];
                double x2 = n2[3], y2 = n2[4];

                double spatialDist = Math.hypot(x2 - x1, y2 - y1);
                if (spatialDist > 0.1) { // Avoid division by very small numbers
                    // Calculate gradient per 10 pixels
                    double gradient = Math.abs(rps2 - rps1) / (spatialDist / 10.0);
                    if (gradient > maxGradient) {
                        maxGradient = gradient;
                        maxGradientPair = new double[]{x1, y1, rps1, x2, y2, rps2, spatialDist};
                    }
                }
            }
        }

        // Log gradient analysis for tuning
        if (maxGradientPair != null) {
            Log.d("LUT_Gradient", String.format(
                "Max gradient: %.3f RPS/10px | Between (%.1f,%.1f)[RPS=%.2f] and (%.1f,%.1f)[RPS=%.2f] | Distance: %.1fpx",
                maxGradient, maxGradientPair[0], maxGradientPair[1], maxGradientPair[2],
                maxGradientPair[3], maxGradientPair[4], maxGradientPair[5], maxGradientPair[6]
            ));
        }

        boolean isSteep = maxGradient > STEEP_GRADIENT_THRESHOLD;
        if (isSteep) {
            Log.w("LUT_Gradient", "STEEP gradient detected! Threshold: " + STEEP_GRADIENT_THRESHOLD);
        }

        return isSteep;
    }

    /**
     * Enhanced prediction with smoothing for steep gradient regions
     * Uses more neighbors with exponential distance weighting
     */
    private Prediction predictWithSmoothing(double xPixel, double yPixel) {
        // Use more neighbors (K=12 instead of 8)
        PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> -a[0]));
        for (Row r : rows) {
            double dx = r.x - xPixel, dy = r.y - yPixel;
            double d = Math.hypot(dx, dy);
            if (pq.size() < SMOOTHING_K) pq.offer(new double[]{d, r.rps, r.hood, r.x, r.y});
            else if (d < pq.peek()[0]) { pq.poll(); pq.offer(new double[]{d, r.rps, r.hood, r.x, r.y}); }
        }

        // Use exponential distance weighting (heavily favor closer points)
        double wsum = 0, rpsSum = 0, hoodSum = 0;
        double scale = 30.0; // Adjust this to control how quickly weight decreases

        // Log neighbor contributions for tuning
        StringBuilder neighborsLog = new StringBuilder("Smoothing neighbors: ");
        List<double[]> neighborsList = new ArrayList<>(pq);
        for (double[] e : neighborsList) {
            double w = Math.exp(-e[0] / scale);
            wsum += w; rpsSum += w * e[1]; hoodSum += w * e[2];
            neighborsLog.append(String.format("(%.1f,%.1f)[d=%.1f,RPS=%.2f,w=%.3f] ",
                e[3], e[4], e[0], e[1], w));
        }
        Log.d("LUT_Smoothing", neighborsLog.toString());

        double predictedRps = rpsSum / wsum;
        double predictedHood = hoodSum / wsum;

        Log.i("LUT_Smoothing", String.format(
            "Query: (%.1f, %.1f) | Smoothed Result: RPS=%.2f, Hood=%.4f | Scale=%s, K=%d",
            xPixel, yPixel, predictedRps, predictedHood, scale, SMOOTHING_K
        ));

        // If hood position is out of bounds, find next best combination
        if (predictedHood < MIN_HOOD_POS || predictedHood > MAX_HOOD_POS) {
            return findBestValidCombination(xPixel, yPixel);
        }

        return new Prediction(predictedRps, predictedHood);
    }

    private Prediction findBestValidCombination(double xPixel, double yPixel) {
        // Find the closest valid combination within hood position bounds
        double bestDist = Double.MAX_VALUE;
        Prediction bestPrediction = null;

        for (Row r : rows) {
            // Only consider rows with valid hood positions
            if (r.hood >= MIN_HOOD_POS && r.hood <= MAX_HOOD_POS) {
                double dx = r.x - xPixel, dy = r.y - yPixel;
                double d = Math.hypot(dx, dy);
                if (d < bestDist) {
                    bestDist = d;
                    bestPrediction = new Prediction(r.rps, r.hood);
                }
            }
        }

        // If no valid combination found, clamp to bounds with original RPS
        if (bestPrediction == null) {
            final int K = 8;
            PriorityQueue<double[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> -a[0]));
            for (Row r : rows) {
                double dx = r.x - xPixel, dy = r.y - yPixel;
                double d = Math.hypot(dx, dy);
                if (pq.size() < K) pq.offer(new double[]{d, r.rps, r.hood});
                else if (d < pq.peek()[0]) { pq.poll(); pq.offer(new double[]{d, r.rps, r.hood}); }
            }
            double wsum = 0, rpsSum = 0;
            while (!pq.isEmpty()) {
                double[] e = pq.poll();
                double w = 1.0 / (e[0] + 1e-6);
                wsum += w; rpsSum += w * e[1];
            }
            double clampedHood = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, 0.5)); // Default to middle
            return new Prediction(rpsSum / wsum, clampedHood);
        }

        return bestPrediction;
    }
}
