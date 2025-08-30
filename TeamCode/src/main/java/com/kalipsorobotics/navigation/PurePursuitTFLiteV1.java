/*
// PurePursuitTFLite.java
// Minimal, FTC-friendly holonomic Pure Pursuit controller that uses a TFLite inverse model.
// I/O conventions:
//   - Waypoints: (x_mm, y_mm, heading_rad)
//   - Pose input: same units (x_mm, y_mm, heading_rad)
//   - Internally converts mm -> m for velocities.
//   - Model input: [vx, vy, omega] in body frame (m/s, m/s, rad/s), normalized by norm.json
//   - Model output: [fL, fR, bL, bR] in [-1, 1]
// Tolerances: 10 mm position, 1 deg heading (≈ 0.01745 rad)
//
// Setup notes:
//   1) Put your trained files in TeamCode/src/main/assets/:
//        - mecanum_inverse.tflite
//        - mecanum_inverse_norm.json
//   2) In TeamCode/build.gradle (Module: TeamCode), add:
//        implementation "org.tensorflow:tensorflow-lite:2.13.0"
//   3) See PurePursuitFollowerOpMode.java for usage example.

package com.kalipsorobotics.navigation;

import android.content.Context;
import android.content.res.AssetManager;

import com.kalipsorobotics.math.Position;

import org.tensorflow.lite.Interpreter;

import java.io.*;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.*;
import org.json.*;

public class PurePursuitTFLite {


    private Pose2d lastFollowPosition = null;

    public static class Pose2d {
        public double x_mm, y_mm, heading_rad;
        public Pose2d(double x_mm, double y_mm, double heading_rad) {
            this.x_mm = x_mm; this.y_mm = y_mm; this.heading_rad = heading_rad;
        }
    }

    public static class MotorPowers {
        public float fL, fR, bL, bR;
        public MotorPowers(float fL, float fR, float bL, float bR) {
            this.fL = fL; this.fR = fR; this.bL = bL; this.bR = bR;
        }
    }

    private static class Norm {
        float[] mean = new float[3];
        float[] std  = new float[3];
    }

    private final Interpreter tflite;
    private final Norm norm;
    private final List<Pose2d> waypoints = new ArrayList<>();
    private int currentSearchWayPointIndex;


    // Parameters (tweak as needed)
    private double maxSpeedMps = 1.5;     // upper bound translation speed
    private double maxOmega = 3.0;        // rad/s
    private double lookaheadMinM = 0.12;  // 12 cm
    private double lookaheadMaxM = 0.45;  // 45 cm
    private double kLookaheadVsSpeed = 0.25; // scales lookahead with speed
    private double endSlowdownRadiusM = 0.30; // start slowing when within 30 cm of final
    private double endSpeedMps = 0.15;    // target near-final speed
    private double kHeading = 2.5;        // P-gain on heading error near the endgame
    private double kXY = 2.0;             // proportional steer toward lookahead (m/s per m error)

    // Tolerances
    private final double xyTolM = 0.010;       // 10 mm
    private final double headingTolRad = Math.toRadians(1.0);

    // Runtime state
    private boolean finished = false;

    public PurePursuitTFLite(Interpreter interpreter, Norm norm) {
        this.tflite = interpreter;
        this.norm = norm;
    }

    // Convenience loader from assets
    public static PurePursuitTFLite fromAssets(Context ctx, String tfliteAsset, String normJsonAsset) throws IOException, JSONException {
        Interpreter interpreter = new Interpreter(loadModelFileFromAssets(ctx.getAssets(), tfliteAsset));
        Norm norm = loadNormFromAssets(ctx.getAssets(), normJsonAsset);
        return new PurePursuitTFLite(interpreter, norm);
    }

    public void setMaxSpeeds(double maxSpeedMps, double maxOmegaRadPerSec) {
        this.maxSpeedMps = maxSpeedMps;
        this.maxOmega = maxOmegaRadPerSec;
    }

    public void setPath(List<Pose2d> pts) {
        waypoints.clear();
        waypoints.addAll(pts);
        finished = false;
    }

    public boolean isFinished() { return finished; }

    // Main update: call each loop with current pose; returns motor powers
    public MotorPowers update(Pose2d current) {
        if (waypoints.size() < 2) {
            finished = true;
            return new MotorPowers(0,0,0,0);
        }
        Pose2d goal = waypoints.get(waypoints.size()-1);

        // Done logic
        double dx = (goal.x_mm - current.x_mm) / 1000.0;
        double dy = (goal.y_mm - current.y_mm) / 1000.0;
        double dPos = Math.hypot(dx, dy);
        double dHead = wrapAngle(goal.heading_rad - current.heading_rad);

        if (dPos <= xyTolM && Math.abs(dHead) <= headingTolRad) {
            finished = true;
            return new MotorPowers(0,0,0,0);
        }

        // Dynamic lookahead based on "intended speed"
        double baseSpeed = maxSpeedMps;
        // Endgame slowdown
        if (dPos < endSlowdownRadiusM) {
            double t = clamp(dPos / endSlowdownRadiusM, 0.0, 1.0);
            baseSpeed = endSpeedMps + t * (maxSpeedMps - endSpeedMps);
        }
        double lookahead = clamp(lookaheadMinM + kLookaheadVsSpeed * baseSpeed, lookaheadMinM, lookaheadMaxM);

        // Find the lookahead point on polyline
        PointM la = findLookaheadPoint(current, lookahead);

        // Desired translation: toward lookahead in robot body frame
        // Current -> LA vector in field frame (meters)
        double laVecX_m = la.x_m - current.x_mm / 1000.0;
        double laVecY_m = la.y_m - current.y_mm / 1000.0;

        // Rotate into body frame (x forward, y left)
        double ch = Math.cos(current.heading_rad), sh = Math.sin(current.heading_rad);
        double vxDir =  ch * laVecX_m + sh * laVecY_m;
        double vyDir = -sh * laVecX_m + ch * laVecY_m;

        // Proportional steer to lookahead, limited to baseSpeed
        double vMag = Math.hypot(vxDir, vyDir);
        double vx, vy;
        if (vMag > 1e-6) {
            double scale = Math.min(baseSpeed, kXY * vMag) / vMag;
            vx = vxDir * scale;
            vy = vyDir * scale;
        } else {
            vx = 0; vy = 0;
        }

        // Heading policy
        double omegaCmd;
        if (dPos < endSlowdownRadiusM) {
            omegaCmd = clamp(kHeading * dHead, -maxOmega, maxOmega);
        } else {
            omegaCmd = clamp(0.5 * dHead, -maxOmega, maxOmega);
        }

        // Invoke inverse model: [vx, vy, omega] -> [fL, fR, bL, bR]
        float[] in = new float[] {
                (float)((vx - norm.mean[0]) / norm.std[0]),
                (float)((vy - norm.mean[1]) / norm.std[1]),
                (float)((omegaCmd - norm.mean[2]) / norm.std[2])
        };
        float[][] out = new float[1][4];
        tflite.run(in, out);

        // Clamp for safety
        float fL = clampF(out[0][0], -1f, 1f);
        float fR = clampF(out[0][1], -1f, 1f);
        float bL = clampF(out[0][2], -1f, 1f);
        float bR = clampF(out[0][3], -1f, 1f);

        return new MotorPowers(fL, fR, bL, bR);
    }

    // ---------- Path & geometry helpers ----------

    private static class PointM { double x_m, y_m; PointM(double x_m, double y_m){this.x_m=x_m;this.y_m=y_m;} }

    private PointM findLookaheadPoint(Pose2d cur, double lookaheadM) {
        // Standard Pure Pursuit: circle (center=current pos, radius=lookahead)
        // with the piecewise-linear path; choose the "forward" intersection.
        double cx = cur.x_mm / 1000.0, cy = cur.y_mm / 1000.0;
        PointM best = new PointM(waypoints.get(0).x_mm/1000.0, waypoints.get(0).y_mm/1000.0);
        double bestDistAlong = 0;

        // Track progress: find nearest segment, then favor intersections ahead
        double progress = nearestProgressAlongPath(cx, cy);

        double accum = 0;
        for (int i=0; i<waypoints.size()-1; i++) {
            Pose2d a = waypoints.get(i);
            Pose2d b = waypoints.get(i+1);
            double ax = a.x_mm/1000.0, ay = a.y_mm/1000.0;
            double bx = b.x_mm/1000.0, by = b.y_mm/1000.0;

            List<PointM> hits = circleSegmentIntersections(cx, cy, lookaheadM, ax, ay, bx, by);
            for (PointM p : hits) {
                double along = accum + Math.hypot(p.x_m-ax, p.y_m-ay);
                if (along > progress && along > bestDistAlong) {
                    bestDistAlong = along;
                    best = p;
                }
            }
            accum += Math.hypot(bx-ax, by-ay);
        }
        return best;
    }

    */
/*private PointM kLookAhead(Pose2d cur, double lookaheadM) {
        for (int i = currentSearchWayPointIndex; i < waypoints.size(); i++) {
            Pose2d currentFollowPosition = waypoints.get(i);

            if (lastFollowPosition == null) {
                currentSearchWayPointIndex = i;
                lastFollowPosition = currentFollowPosition;
                PointM pointM = new PointM(currentFollowPosition.x_mm, currentFollowPosition.y_mm);
                return pointM;
            }
            double distanceToFollow = Math.hypot(cur.x_mm - currentFollowPosition.x_mm, cur.y_mm - currentFollowPosition.y_mm);
            if (distanceToFollow > lookaheadM * 1000) {
                currentSearchWayPointIndex = i;
                lastFollowPosition = currentFollowPosition;
                PointM pointM = new PointM(currentFollowPosition.x_mm, currentFollowPosition.y_mm);
                return pointM;
            }*//*
*/
/* else {
                if (Math.abs(currentPosition.getTheta() - currentFollowPosition.getTheta()) > PATH_ANGLE_TOLERANCE) {
                    currentSearchWayPointIndex = i;
                    return Optional.of(currentFollowPosition);
                }
            }*//*
*/
/*
        }
        return null;

    }*//*


    private double nearestProgressAlongPath(double x_m, double y_m) {
        // Returns arc-length distance along path to nearest projection
        double accum = 0, best = 0, bestDist = Double.POSITIVE_INFINITY;
        for (int i=0; i<waypoints.size()-1; i++) {
            double ax = waypoints.get(i).x_mm/1000.0, ay = waypoints.get(i).y_mm/1000.0;
            double bx = waypoints.get(i+1).x_mm/1000.0, by = waypoints.get(i+1).y_mm/1000.0;
            double[] proj = projectPointToSegment(x_m, y_m, ax, ay, bx, by);
            double px = proj[0], py = proj[1], t = proj[2];
            double d = Math.hypot(px-x_m, py-y_m);
            if (d < bestDist) {
                bestDist = d;
                best = accum + t * Math.hypot(bx-ax, by-ay);
            }
            accum += Math.hypot(bx-ax, by-ay);
        }
        return best;
    }

    private static double[] projectPointToSegment(double x, double y, double ax, double ay, double bx, double by) {
        double vx = bx-ax, vy = by-ay;
        double ux = x-ax, uy = y-ay;
        double len2 = vx*vx + vy*vy + 1e-12;
        double t = clamp((ux*vx + uy*vy)/len2, 0.0, 1.0);
        return new double[]{ ax + t*vx, ay + t*vy, t };
    }

    private static List<PointM> circleSegmentIntersections(double cx, double cy, double r,
                                                           double ax, double ay, double bx, double by) {
        // Line parametric: P = A + t*(B-A), t in [0,1]
        double dx = bx-ax, dy = by-ay;
        double fx = ax-cx, fy = ay-cy;

        double a = dx*dx + dy*dy;
        double b = 2*(fx*dx + fy*dy);
        double c = fx*fx + fy*fy - r*r;

        double disc = b*b - 4*a*c;
        ArrayList<PointM> res = new ArrayList<>();
        if (disc < 0) return res;

        double s = Math.sqrt(disc);
        double t1 = (-b - s) / (2*a);
        double t2 = (-b + s) / (2*a);

        if (t1 >= 0 && t1 <= 1) res.add(new PointM(ax + t1*dx, ay + t1*dy));
        if (t2 >= 0 && t2 <= 1) res.add(new PointM(ax + t2*dx, ay + t2*dy));
        return res;
    }

    // ---------- Utils ----------

    private static double wrapAngle(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private static float clampF(float v, float lo, float hi) {
        return (float) Math.max(lo, Math.min(hi, v));
    }

    // ---------- TFLite & Norm loaders ----------

    private static MappedByteBuffer loadModelFileFromAssets(AssetManager am, String assetPath) throws IOException {
        try (InputStream is = am.open(assetPath);
             FileChannel channel = streamToTempFileChannel(is)) {
            return channel.map(FileChannel.MapMode.READ_ONLY, 0, channel.size());
        }
    }

    private static FileChannel streamToTempFileChannel(InputStream is) throws IOException {
        File temp = File.createTempFile("tflite", "model");
        try (FileOutputStream fos = new FileOutputStream(temp)) {
            byte[] buf = new byte[1<<16];
            int r;
            while ((r = is.read(buf)) != -1) fos.write(buf, 0, r);
        }
        RandomAccessFile raf = new RandomAccessFile(temp, "r");
        return raf.getChannel();
    }

    private static Norm loadNormFromAssets(AssetManager am, String normAsset) throws IOException, JSONException {
        try (InputStream is = am.open(normAsset)) {
            String json = readAll(is);
            JSONObject obj = new JSONObject(json);
            Norm n = new Norm();
            JSONArray m = obj.getJSONArray("x_mean");
            JSONArray s = obj.getJSONArray("x_std");
            for (int i=0;i<3;i++) {
                n.mean[i] = (float)m.getDouble(i);
                n.std[i]  = (float)s.getDouble(i);
                if (Math.abs(n.std[i]) < 1e-8) n.std[i] = 1f;
            }
            return n;
        }
    }

    private static String readAll(InputStream is) throws IOException {
        ByteArrayOutputStream bos = new ByteArrayOutputStream();
        byte[] buf = new byte[8192]; int r;
        while ((r = is.read(buf)) != -1) bos.write(buf, 0, r);
        return bos.toString("UTF-8");
    }
}
*/
