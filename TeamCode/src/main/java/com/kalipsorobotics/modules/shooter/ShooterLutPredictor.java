package com.kalipsorobotics.modules.shooter;

import android.content.Context;
import android.content.res.AssetManager;
import org.json.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;

public class ShooterLutPredictor {
    private static final double MIN_HOOD_POS = 0.23;
    private static final double MAX_HOOD_POS = 0.8;

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
        // KNN (K=8), inverse-distance weighting (Shepard). If exact hit, return it.
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
            if (pq.size() < K) pq.offer(new double[]{d, r.rps, r.hood});
            else if (d < pq.peek()[0]) { pq.poll(); pq.offer(new double[]{d, r.rps, r.hood}); }
        }

        // Calculate weighted average prediction
        double wsum = 0, rpsSum = 0, hoodSum = 0;
        while (!pq.isEmpty()) {
            double[] e = pq.poll();
            double w = 1.0 / (e[0] + 1e-6);
            wsum += w; rpsSum += w * e[1]; hoodSum += w * e[2];
        }
        double predictedRps = rpsSum / wsum;
        double predictedHood = hoodSum / wsum;

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
