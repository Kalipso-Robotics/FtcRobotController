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
        notifyAll();
    }

    /**
     * Blocks until a sample newer than afterNanos exists or timeoutMs passes.
     * @return true if a newer sample exists
     */
    public synchronized boolean awaitNewer(long afterNanos, long timeoutMs) throws InterruptedException {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        while (size == 0 || nanos[newest()] <= afterNanos) {
            long leftMs = (deadline - System.nanoTime()) / 1_000_000L;
            if (leftMs <= 0) return false;
            wait(leftMs);
        }
        return true;
    }

    /** Newest pose with velocity and acceleration (mm, rad, ms) from a least-squares fit. */
    public static class Motion {
        public final long sampleNanos;
        public final double x, y, theta;
        public final double vx, vy, omega;
        public final double ax, ay, alpha;

        public Motion(long sampleNanos, double x, double y, double theta,
                      double vx, double vy, double omega, double ax, double ay, double alpha) {
            this.sampleNanos = sampleNanos;
            this.x = x; this.y = y; this.theta = theta;
            this.vx = vx; this.vy = vy; this.omega = omega;
            this.ax = ax; this.ay = ay; this.alpha = alpha;
        }
    }

    /**
     * Fits y(s) = y0 + v*s + a*s^2/2 per axis over the samples within windowNanos of the newest
     * (s in ms, 0 at the newest). Quadratic with 3+ samples, linear with 2, zero motion with 1.
     * Heading is unwrapped about the newest. Null if empty. Position is the raw newest sample.
     */
    public Motion fit(long windowNanos) {
        // Accumulate the normal equations under the lock (no allocation, ~n multiply-adds);
        // the 3x3 solve runs outside it so odometry's record() is never held up by the math.
        double s1 = 0, s2 = 0, s3 = 0, s4 = 0; // sums of s^k (basis b = {1, s, s^2/2})
        double[] rx = new double[3], ry = new double[3], rt = new double[3];
        int n = 0;
        long newestNanos;
        double nx, ny, nt;
        synchronized (this) {
            if (size == 0) return null;
            int newest = newest();
            newestNanos = nanos[newest];
            nx = xs[newest]; ny = ys[newest]; nt = thetas[newest];
            for (int k = 0; k < size; k++) {
                int i = index(k);
                if (k > 0 && newestNanos - nanos[i] > windowNanos) break;
                double s = (nanos[i] - newestNanos) / 1e6, h = 0.5 * s * s;
                double th = nt + MathFunctions.angleWrapRad(thetas[i] - nt);
                s1 += s; s2 += s * s; s3 += s * s * s; s4 += s * s * s * s;
                rx[0] += xs[i]; rx[1] += s * xs[i]; rx[2] += h * xs[i];
                ry[0] += ys[i]; ry[1] += s * ys[i]; ry[2] += h * ys[i];
                rt[0] += th;    rt[1] += s * th;    rt[2] += h * th;
                n++;
            }
        }
        double[] fx, fy, ft;
        if (n < 2) {
            fx = fy = ft = new double[]{0, 0, 0};
        } else {
            // Gram matrix of {1, s, s^2/2}
            double[][] g = {{n, s1, 0.5 * s2}, {s1, s2, 0.5 * s3}, {0.5 * s2, 0.5 * s3, 0.25 * s4}};
            double[][] r = {rx, ry, rt};
            double[][] f = n >= 3 ? solve(g, 3, r) : null;
            if (f == null) f = solve(g, 2, r); // linear with 2 samples or a singular quadratic
            if (f == null) f = new double[][]{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
            fx = f[0]; fy = f[1]; ft = f[2];
        }
        return new Motion(newestNanos, nx, ny, nt, fx[1], fy[1], ft[1], fx[2], fy[2], ft[2]);
    }

    /**
     * Solves the leading deg x deg block of g against each right-hand side in rhs (Gaussian
     * elimination, partial pivoting, one factorisation for all). Returns {x,y,theta} coefficient
     * triples (unused slots 0), or null if singular.
     */
    private static double[][] solve(double[][] g, int deg, double[][] rhs) {
        int nr = rhs.length;
        double[][] m = new double[deg][deg + nr];
        for (int r = 0; r < deg; r++) {
            for (int c = 0; c < deg; c++) m[r][c] = g[r][c];
            for (int k = 0; k < nr; k++) m[r][deg + k] = rhs[k][r];
        }
        for (int c = 0; c < deg; c++) {
            int piv = c;
            for (int r = c + 1; r < deg; r++) if (Math.abs(m[r][c]) > Math.abs(m[piv][c])) piv = r;
            double[] t = m[c]; m[c] = m[piv]; m[piv] = t;
            if (Math.abs(m[c][c]) < 1e-12) return null;
            for (int r = c + 1; r < deg; r++) {
                double f = m[r][c] / m[c][c];
                for (int k = c; k < deg + nr; k++) m[r][k] -= f * m[c][k];
            }
        }
        double[][] out = new double[nr][3];
        for (int k = 0; k < nr; k++) {
            for (int r = deg - 1; r >= 0; r--) {
                double v = m[r][deg + k];
                for (int c = r + 1; c < deg; c++) v -= m[r][c] * out[k][c];
                out[k][r] = v / m[r][r];
            }
        }
        return out;
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
        int k = 0;
        while (nanos[index(k)] > queryNanos) k++;
        int lo = index(k);
        int hi = (lo + 1) % nanos.length;
        double w = (double) (queryNanos - nanos[lo]) / (nanos[hi] - nanos[lo]);
        return new Position(
                xs[lo] + w * (xs[hi] - xs[lo]),
                ys[lo] + w * (ys[hi] - ys[lo]),
                MathFunctions.angleWrapRad(
                        thetas[lo] + w * MathFunctions.angleWrapRad(thetas[hi] - thetas[lo])));
    }

    /** Slot of the k-th newest sample (0 = newest). */
    private int index(int k) {
        return (next - 1 - k + nanos.length) % nanos.length;
    }

    private int newest() {
        return (next - 1 + nanos.length) % nanos.length;
    }

    private Position pose(int i) {
        return new Position(xs[i], ys[i], thetas[i]);
    }
}
