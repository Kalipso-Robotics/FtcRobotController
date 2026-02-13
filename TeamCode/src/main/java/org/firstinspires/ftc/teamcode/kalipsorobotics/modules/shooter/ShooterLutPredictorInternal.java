package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import android.content.Context;
import android.content.res.AssetManager;
import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

public class ShooterLutPredictorInternal {
    private static final double MIN_HOOD_POS = 0.23;
    private static final double MAX_HOOD_POS = 0.8;

    public static class Prediction {
        public final double rps;
        public final double hood;
        public Prediction(double rps, double hood) { this.rps = rps; this.hood = hood; }

        @Override
        public String toString() {
            return "Prediction(rps=" + rps + ", hood=" + hood + ")";
        }
    }
    private static class Row {
        final double x, y, rps, hood;
        Row(double x, double y, double rps, double hood) { this.x=x; this.y=y; this.rps=rps; this.hood=hood; }
    }
    private final List<Row> rows = new ArrayList<>();

    public ShooterLutPredictorInternal(Context ctx, String assetName) throws IOException {
        // Load binary LUT data (much faster than JSON parsing)
        AssetManager am = ctx.getAssets();
        try (InputStream is = am.open(assetName)) {
            // Read entire file into byte array
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            byte[] buffer = new byte[8192];
            int bytesRead;
            while ((bytesRead = is.read(buffer)) != -1) {
                baos.write(buffer, 0, bytesRead);
            }

            // Parse binary format
            ByteBuffer bb = ByteBuffer.wrap(baos.toByteArray());
            bb.order(ByteOrder.LITTLE_ENDIAN);

            // Read number of entries
            int numEntries = bb.getInt();

            // Read entries (each entry is 4 doubles = 32 bytes)
            for (int i = 0; i < numEntries; i++) {
                double x = bb.getDouble();
                double y = bb.getDouble();
                double rps = bb.getDouble();
                double hood = bb.getDouble();
                rows.add(new Row(x, y, rps, hood));
            }
        }
    }

    // Constructor for unit tests that read from file system
    public ShooterLutPredictorInternal(InputStream is) throws IOException {
        // Read entire file into byte array
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        byte[] buffer = new byte[8192];
        int bytesRead;
        while ((bytesRead = is.read(buffer)) != -1) {
            baos.write(buffer, 0, bytesRead);
        }

        // Parse binary format
        ByteBuffer bb = ByteBuffer.wrap(baos.toByteArray());
        bb.order(ByteOrder.LITTLE_ENDIAN);

        // Read number of entries
        int numEntries = bb.getInt();

        // Read entries (each entry is 4 doubles = 32 bytes)
        for (int i = 0; i < numEntries; i++) {
            double x = bb.getDouble();
            double y = bb.getDouble();
            double rps = bb.getDouble();
            double hood = bb.getDouble();
            rows.add(new Row(x, y, rps, hood));
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
                double clampedHood = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, r.hood));
                return new Prediction(r.rps, clampedHood);
            }
            if (pq.size() < K) pq.offer(new double[]{d, r.rps, r.hood});
            else if (d < pq.peek()[0]) { pq.poll(); pq.offer(new double[]{d, r.rps, r.hood}); }
        }
        double wsum = 0, rpsSum = 0, hoodSum = 0;
        while (!pq.isEmpty()) {
            double[] e = pq.poll();
            double w = 1.0 / (e[0] + 1e-6);
            wsum += w; rpsSum += w * e[1]; hoodSum += w * e[2];
        }
        double predictedRps = rpsSum / wsum;
        double predictedHood = hoodSum / wsum;

        // Clamp hood position to valid bounds
        double clampedHood = Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, predictedHood));

        return new Prediction(predictedRps, clampedHood);
    }
}
