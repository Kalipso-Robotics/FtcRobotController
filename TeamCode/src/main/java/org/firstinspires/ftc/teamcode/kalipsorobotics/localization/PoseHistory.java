package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

/**
 * Fixed-size ring buffer of timestamped robot poses, so a camera frame can be turned into field
 * coordinates with the pose it was captured at rather than the latest one. Odometry records on
 * its executor thread while actions read from the main thread, hence synchronized.
 *
 * Times are System.nanoTime() values. 128 samples at a ~5-15 ms odometry tick is at least 0.6 s.
 */
public class PoseHistory {

    private final long[] nanos;
    private final double[] xs, ys, thetas;
    private int size = 0;
    private int next = 0; // slot the next sample goes into

    public PoseHistory(int capacity) {
        nanos = new long[capacity];
        xs = new double[capacity];
        ys = new double[capacity];
        thetas = new double[capacity];
    }

    /** Stores a copy of p. A sample not newer than the last one is ignored. */
    public synchronized void record(long sampleNanos, Position p) {
        if (size > 0 && sampleNanos <= nanos[newest()]) return;
        nanos[next] = sampleNanos;
        xs[next] = p.getX();
        ys[next] = p.getY();
        thetas[next] = p.getTheta();
        next = (next + 1) % nanos.length;
        if (size < nanos.length) size++;
    }

    /**
     * Pose at the given time: null if the buffer is empty or the time is older than the oldest
     * sample (too stale to trust), the newest sample if the time is at or after it, otherwise a
     * linear interpolation with the heading blended along the short way round.
     */
    public synchronized Position at(long queryNanos) {
        if (size == 0) return null;
        int oldest = (next - size + nanos.length) % nanos.length;
        if (queryNanos < nanos[oldest]) return null;
        int newest = newest();
        if (queryNanos >= nanos[newest]) return pose(newest);

        // Walk back from the newest until the sample at or before the query.
        int lo = newest;
        while (nanos[lo] > queryNanos) {
            lo = (lo - 1 + nanos.length) % nanos.length;
        }
        int hi = (lo + 1) % nanos.length;
        double w = (double) (queryNanos - nanos[lo]) / (nanos[hi] - nanos[lo]);
        return new Position(
                xs[lo] + w * (xs[hi] - xs[lo]),
                ys[lo] + w * (ys[hi] - ys[lo]),
                MathFunctions.angleWrapRad(
                        thetas[lo] + w * MathFunctions.angleWrapRad(thetas[hi] - thetas[lo])));
    }

    private int newest() {
        return (next - 1 + nanos.length) % nanos.length;
    }

    private Position pose(int i) {
        return new Position(xs[i], ys[i], thetas[i]);
    }
}
