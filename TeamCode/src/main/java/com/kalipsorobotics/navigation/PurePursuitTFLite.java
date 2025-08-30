// PurePursuitTFLite.java
// Minimal, FTC-friendly holonomic Pure Pursuit controller that uses a TFLite inverse model.
// I/O conventions:
//   - Waypoints: (x_mm, y_mm, heading_rad)
//   - Pose input: same units (x_mm, y_mm, heading_rad)
//   - Internally converts mm -> m for velocities.
//   - Model input: [vx, vy, omega] in body frame (m/s, m/s, rad/s), normalized by norm.json
//   - Model output: [fL, fR, bL, bR] in [-1, 1]
// Design: Fast waypoint advancement, high precision only at final goal
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
    private int currentTargetIndex = 1; // Start targeting waypoint 1

    // Parameters optimized for waypoint skipping but final precision
    private double maxSpeedMps = 1.0;
    private double maxOmega = 2.5;
    private double lookaheadMinM = 0.3;   // larger minimum lookahead
    private double lookaheadMaxM = 1.2;   // much larger maximum
    private double kLookaheadVsSpeed = 0.6;

    // Very aggressive waypoint advancement - only care about final goal
    private double waypointSkipRadiusM = 0.1; // 40 cm - skip waypoints aggressively
    private double finalGoalRadiusM = 0.025; // 25 mm for final goal - high precision
    private double finalGoalSlowdownRadiusM = 0.1; // start slowing when close to final
    private double finalSpeedMps = 0.1;   // very slow for final approach

    private double kHeading = 3.0;
    private double kXY = 1.8;

    // Tight tolerances only for final goal
    private final double xyTolM = 0.025;       // 25 mm - high precision
    private final double headingTolRad = Math.toRadians(3.0); // 3 degrees

    private boolean finished = false;
    private boolean isAtFinalWaypoint = false;

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
        currentTargetIndex = Math.min(1, pts.size() - 1);
        finished = false;
        isAtFinalWaypoint = false;
    }

    public boolean isFinished() { return finished; }

    // Main update: call each loop with current pose; returns motor powers
    public MotorPowers update(Pose2d current) {
        if (waypoints.size() < 2) {
            finished = true;
            return new MotorPowers(0,0,0,0);
        }

        // Convert current position to meters
        double curX_m = current.x_mm / 1000.0;
        double curY_m = current.y_mm / 1000.0;

        // Aggressively advance through waypoints (except final)
        updateTargetWaypoint(curX_m, curY_m);

        Pose2d goal = waypoints.get(waypoints.size()-1);
        double goalX_m = goal.x_mm / 1000.0;
        double goalY_m = goal.y_mm / 1000.0;

        // Check distance to final goal
        double dxGoal = goalX_m - curX_m;
        double dyGoal = goalY_m - curY_m;
        double dPosGoal = Math.hypot(dxGoal, dyGoal);
        double dHead = wrapAngle(goal.heading_rad - current.heading_rad);

        // Check if we're at the final waypoint
        isAtFinalWaypoint = (currentTargetIndex >= waypoints.size() - 1);

        // Final completion check - only apply tight tolerances at the very end
        if (isAtFinalWaypoint && dPosGoal <= xyTolM && Math.abs(dHead) <= headingTolRad) {
            finished = true;
            return new MotorPowers(0,0,0,0);
        }

        // Calculate target point for lookahead
        PointM targetPoint = findTargetPoint(curX_m, curY_m);

        // Calculate desired velocities in body frame
        double laVecX_m = targetPoint.x_m - curX_m;
        double laVecY_m = targetPoint.y_m - curY_m;

        // Rotate to body frame (x forward, y left)
        double ch = Math.cos(current.heading_rad), sh = Math.sin(current.heading_rad);
        double vxDir =  ch * laVecX_m + sh * laVecY_m;
        double vyDir = -sh * laVecX_m + ch * laVecY_m;

        // Speed control - slow down only near final goal
        double targetSpeed = maxSpeedMps;
        if (isAtFinalWaypoint && dPosGoal < finalGoalSlowdownRadiusM) {
            double t = clamp(dPosGoal / finalGoalSlowdownRadiusM, 0.0, 1.0);
            targetSpeed = finalSpeedMps + t * (maxSpeedMps - finalSpeedMps);
        }

        // Scale to desired speed
        double vMag = Math.hypot(vxDir, vyDir);
        double vx, vy;
        if (vMag > 1e-6) {
            double scale = Math.min(targetSpeed, kXY * vMag) / vMag;
            vx = vxDir * scale;
            vy = vyDir * scale;
        } else {
            vx = 0; vy = 0;
        }

        // Heading control - only care about final heading at the very end
        double omegaCmd;
        if (isAtFinalWaypoint) {
            // Strong heading correction only at final waypoint
            omegaCmd = clamp(kHeading * dHead, -maxOmega, maxOmega);
        } else {
            // Minimal heading correction during path following
            omegaCmd = clamp(0.3 * dHead, -maxOmega * 0.3, maxOmega * 0.3);
        }

        // Invoke inverse model
        float[] in = new float[] {
                (float)((vx - norm.mean[0]) / norm.std[0]),
                (float)((vy - norm.mean[1]) / norm.std[1]),
                (float)((omegaCmd - norm.mean[2]) / norm.std[2])
        };
        float[][] out = new float[1][4];
        tflite.run(in, out);

        // Clamp outputs
        float fL = clampF(out[0][0], -1f, 1f);
        float fR = clampF(out[0][1], -1f, 1f);
        float bL = clampF(out[0][2], -1f, 1f);
        float bR = clampF(out[0][3], -1f, 1f);

        return new MotorPowers(fL, fR, bL, bR);
    }

    // ---------- Simplified waypoint advancement ----------

    private void updateTargetWaypoint(double curX_m, double curY_m) {
        // Skip through waypoints aggressively until we reach the final one
        while (currentTargetIndex < waypoints.size() - 1) {
            Pose2d target = waypoints.get(currentTargetIndex);
            double targetX_m = target.x_mm / 1000.0;
            double targetY_m = target.y_mm / 1000.0;

            double distToTarget = Math.hypot(targetX_m - curX_m, targetY_m - curY_m);

            // Very aggressive advancement - skip waypoints we're reasonably close to
            if (distToTarget < waypointSkipRadiusM) {
                currentTargetIndex++;
            } else {
                break;
            }
        }

        // Force to final waypoint if we're close to the goal
        Pose2d finalGoal = waypoints.get(waypoints.size() - 1);
        double finalX_m = finalGoal.x_mm / 1000.0;
        double finalY_m = finalGoal.y_mm / 1000.0;
        double distToFinal = Math.hypot(finalX_m - curX_m, finalY_m - curY_m);

        if (distToFinal < finalGoalSlowdownRadiusM) {
            currentTargetIndex = waypoints.size() - 1;
        }
    }

    private static class PointM {
        double x_m, y_m;
        PointM(double x_m, double y_m){this.x_m=x_m;this.y_m=y_m;}
    }

    private PointM findTargetPoint(double curX_m, double curY_m) {
        // Simple strategy: if not at final waypoint, target the next waypoint
        // If at final waypoint, use high precision targeting

        if (currentTargetIndex >= waypoints.size()) {
            // Use final goal
            Pose2d goal = waypoints.get(waypoints.size() - 1);
            return new PointM(goal.x_mm / 1000.0, goal.y_mm / 1000.0);
        }

        Pose2d target = waypoints.get(currentTargetIndex);
        double targetX_m = target.x_mm / 1000.0;
        double targetY_m = target.y_mm / 1000.0;

        // If we're at the final waypoint, target exactly
        if (isAtFinalWaypoint) {
            return new PointM(targetX_m, targetY_m);
        }

        // For intermediate waypoints, use a lookahead approach
        double distToTarget = Math.hypot(targetX_m - curX_m, targetY_m - curY_m);
        double lookahead = clamp(lookaheadMinM + kLookaheadVsSpeed * maxSpeedMps, lookaheadMinM, lookaheadMaxM);

        // If target is closer than lookahead, extend toward next waypoint
        if (distToTarget < lookahead && currentTargetIndex < waypoints.size() - 1) {
            Pose2d nextTarget = waypoints.get(currentTargetIndex + 1);
            double nextX_m = nextTarget.x_mm / 1000.0;
            double nextY_m = nextTarget.y_mm / 1000.0;

            // Interpolate between current target and next target
            double totalDist = Math.hypot(nextX_m - targetX_m, nextY_m - targetY_m);
            if (totalDist > 1e-6) {
                double remaining = lookahead - distToTarget;
                double ratio = Math.min(remaining / totalDist, 1.0);

                double extendedX = targetX_m + ratio * (nextX_m - targetX_m);
                double extendedY = targetY_m + ratio * (nextY_m - targetY_m);

                return new PointM(extendedX, extendedY);
            }
        }

        return new PointM(targetX_m, targetY_m);
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

    // Public methods for debugging
    public int getCurrentTargetIndex() { return currentTargetIndex; }
    public int getTotalWaypoints() { return waypoints.size(); }
    public boolean isAtFinalWaypoint() { return isAtFinalWaypoint; }

    public Pose2d getCurrentTarget() {
        if (currentTargetIndex < waypoints.size()) {
            return waypoints.get(currentTargetIndex);
        }
        return waypoints.get(waypoints.size() - 1);
    }

    // Get distance to current target for debugging
    public double getDistanceToCurrentTarget(Pose2d current) {
        Pose2d target = getCurrentTarget();
        return Math.hypot(target.x_mm - current.x_mm, target.y_mm - current.y_mm);
    }
}