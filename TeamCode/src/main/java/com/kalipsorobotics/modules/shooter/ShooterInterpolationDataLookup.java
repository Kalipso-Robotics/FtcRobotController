package com.kalipsorobotics.modules.shooter;

import java.util.*;

/**
 * Linear interpolation lookup for shooter parameters based on manually tuned test data.
 * Data is hard-coded from ShooterTestedManualData.csv for performance.
 */
public class ShooterInterpolationDataLookup implements IShooterPredictor {
    // No clamping - use the exact values from the manual tuning data
    private static final double HOOD_OFFSET = 0.25;
    private static final double MIN_HOOD_POS = 0.0;
    private static final double MAX_HOOD_POS = 0.55;

    private static class DataPoint {
        final double distanceMM;
        final double rps;
        final double hoodPosition;

        DataPoint(double distanceMM, double rps, double hoodPosition) {
            this.distanceMM = distanceMM;
            this.rps = rps;
            this.hoodPosition = hoodPosition;
        }
    }

    // Static data from ShooterTestedManualData.csv
    // Format: DistanceMM, RPS, Hood Position Relative
    private static final ArrayList<DataPoint> DATA_POINTS = new ArrayList<DataPoint>() {{
        add(new DataPoint(540, 34, 0.1));
        add(new DataPoint(682, 34, 0.31));
        add(new DataPoint(913, 35, 0.38));
        add(new DataPoint(1108, 36, 0.55));
        add(new DataPoint(1323, 37, 0.55));
        add(new DataPoint(1580, 38, 0.55));
        add(new DataPoint(1698, 39, 0.55));
        add(new DataPoint(1808, 40, 0.55));
        add(new DataPoint(1955, 41, 0.55));
        add(new DataPoint(2057, 42, 0.55));
        add(new DataPoint(2179, 44, 0.55));
        add(new DataPoint(2330, 45, 0.55));
        add(new DataPoint(2516, 46, 0.55));
        add(new DataPoint(2639, 48, 0.55));
        add(new DataPoint(2825, 49, 0.55));
        add(new DataPoint(2952, 50, 0.55));
        add(new DataPoint(3076, 50.5, 0.55));
        add(new DataPoint(3304, 51.5, 0.55));
        add(new DataPoint(3389, 51.5, 0.55));
        add(new DataPoint(3547, 52, 0.55));
    }};

    /**
     * Default constructor - uses static data.
     */
    public ShooterInterpolationDataLookup() {
        // Data is already loaded statically
    }

    /**
     * Get shooter parameters for a given distance using linear interpolation.
     *
     * @param distanceMM Distance to target in millimeters
     * @return Shooter parameters (RPS and hood position)
     */
    @Override
    public IShooterPredictor.ShooterParams predict(double distanceMM) {
        // If distance is below minimum, return first point
        if (distanceMM <= DATA_POINTS.get(0).distanceMM) {
            DataPoint dp = DATA_POINTS.get(0);
            double clampedHood = clampHood(dp.hoodPosition);
            clampedHood += HOOD_OFFSET;
            return new IShooterPredictor.ShooterParams(dp.rps, clampedHood);
        }

        // If distance is above maximum, return last point
        if (distanceMM >= DATA_POINTS.get(DATA_POINTS.size() - 1).distanceMM) {
            DataPoint dp = DATA_POINTS.get(DATA_POINTS.size() - 1);
            double clampedHood = clampHood(dp.hoodPosition);
            clampedHood += HOOD_OFFSET;
            return new IShooterPredictor.ShooterParams(dp.rps, clampedHood);
        }

        // Find the two surrounding points
        for (int i = 0; i < DATA_POINTS.size() - 1; i++) {
            DataPoint lower = DATA_POINTS.get(i);
            DataPoint upper = DATA_POINTS.get(i + 1);

            if (distanceMM >= lower.distanceMM && distanceMM <= upper.distanceMM) {
                // Linear interpolation
                double t = (distanceMM - lower.distanceMM) / (upper.distanceMM - lower.distanceMM);

                double interpolatedRPS = lower.rps + t * (upper.rps - lower.rps);
                double interpolatedHood = lower.hoodPosition + t * (upper.hoodPosition - lower.hoodPosition);

                double clampedHood = clampHood(interpolatedHood);
                clampedHood += HOOD_OFFSET;
                return new IShooterPredictor.ShooterParams(interpolatedRPS, clampedHood);
            }
        }

        // Should never reach here, but return last point as fallback
        DataPoint dp = DATA_POINTS.get(DATA_POINTS.size() - 1);
        double clampedHood = clampHood(dp.hoodPosition);
        clampedHood += HOOD_OFFSET;
        return new IShooterPredictor.ShooterParams(dp.rps, clampedHood);
    }

    /**
     * Clamp hood position to valid range.
     */
    private double clampHood(double hood) {
        return Math.max(MIN_HOOD_POS, Math.min(MAX_HOOD_POS, hood));
    }

    /**
     * Get the minimum distance in the dataset.
     */
    public double getMinDistance() {
        return DATA_POINTS.get(0).distanceMM;
    }

    /**
     * Get the maximum distance in the dataset.
     */
    public double getMaxDistance() {
        return DATA_POINTS.get(DATA_POINTS.size() - 1).distanceMM;
    }

    /**
     * Get the number of data points loaded.
     */
    public int getDataPointCount() {
        return DATA_POINTS.size();
    }
}
