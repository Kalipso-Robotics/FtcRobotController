package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeNotNull;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Matrix;
import org.junit.Test;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Offline mount fit. Run the RaytracingGroundTruth OpMode, pull its CSV, drop it in
 * TeamCode/src/test/resources/raytracing/RaytracingGroundTruth.csv and run this test:
 *
 *   ./gradlew :TeamCode:testDebugUnitTest --tests '*RaytracerFitTest*' -i
 *
 * It grid-searches pitch/yaw/roll through the real Raytracer.estimate (coarse 0.5 deg, then fine
 * 0.1 deg) and prints the best mount with its forward/lateral RMS. Paste it into
 * VisionConfig.ARDUCAM. Skipped when the CSV is absent.
 *
 * Needs off-centre shots: with centred rows only, yaw and roll are not identifiable and
 * the search just trades them against pitch. The lens (fx, fy, cx, cy) is NOT fitted here, on
 * purpose -- see VisionConfig; fitting it to tape-measured distances launders mount error into
 * the lens model.
 */
public class RaytracerFitTest {

    private static final String CSV = "/raytracing/RaytracingGroundTruth.csv";

    /** One shot: per-shot median edges (robust to a bad frame in the burst) and the tape truth. */
    private static class Shot {
        double fwd, lat, uL, uR, vT, vB, radius;
    }

    private static double median(List<Double> v) {
        double[] a = v.stream().mapToDouble(Double::doubleValue).toArray();
        Arrays.sort(a);
        return a.length % 2 == 1 ? a[a.length / 2] : (a[a.length / 2 - 1] + a[a.length / 2]) / 2;
    }

    private static List<Shot> load(InputStream in) throws Exception {
        BufferedReader br = new BufferedReader(new InputStreamReader(in));
        String[] head = br.readLine().split(",");
        List<String> cols = Arrays.asList(head);
        int iD = cols.indexOf("KnownDistMM"), iLat = cols.indexOf("KnownLateralMM"), iLabel = cols.indexOf("Label");
        int iL = cols.indexOf("EdgeL"), iR = cols.indexOf("EdgeR"), iT = cols.indexOf("EdgeT"), iB = cols.indexOf("EdgeB");
        Map<String, List<String[]>> byShot = new LinkedHashMap<>();
        for (String line; (line = br.readLine()) != null; ) {
            String[] f = line.split(",");
            if (f.length < head.length) continue;
            byShot.computeIfAbsent(f[iD] + "/" + f[iLat], k -> new ArrayList<>()).add(f);
        }
        List<Shot> shots = new ArrayList<>();
        for (List<String[]> rows : byShot.values()) {
            Shot s = new Shot();
            s.fwd = Double.parseDouble(rows.get(0)[iD]);
            s.lat = Double.parseDouble(rows.get(0)[iLat]);
            String label = rows.get(0)[iLabel].toLowerCase(Locale.US);
            boolean pollen = label.equals("pollen") || label.equals("yellow");
            s.radius = (pollen ? VisionConfig.POLLEN_DIAMETER_MM : VisionConfig.NECTAR_DIAMETER_MM) / 2.0;
            double[][] v = new double[4][rows.size()];
            int[] idx = {iL, iR, iT, iB};
            List<List<Double>> col = new ArrayList<>();
            for (int k = 0; k < 4; k++) {
                List<Double> l = new ArrayList<>();
                for (String[] r : rows) l.add(Double.parseDouble(r[idx[k]]));
                col.add(l);
            }
            s.uL = median(col.get(0));
            s.uR = median(col.get(1));
            s.vT = median(col.get(2));
            s.vB = median(col.get(3));
            shots.add(s);
        }
        return shots;
    }

    /** Sum of squared forward and lateral error over all shots; shots that don't raytrace cost 1e6. */
    private static double cost(Raytracer rt, List<Shot> shots, VisionConfig.Mount m) {
        double se = 0;
        for (Shot s : shots) {
            Raytracer.Estimate e = rt.estimate(s.uL, s.uR, s.vT, s.vB, s.radius, m);
            se += e == null ? 1e6 : Math.pow(e.robotPos.getY() - s.fwd, 2) + Math.pow(e.robotPos.getX() - s.lat, 2);
        }
        return se;
    }

    /** Best {pitch, yaw, roll} for the shots, searching around the current mount; also returns its cost. */
    private static double[] fit(Raytracer rt, List<Shot> shots, VisionConfig.Mount cur) {
        double[] best = {cur.pitchDeg, cur.yawDeg, cur.rollDeg};
        double bestCost = cost(rt, shots, cur);

        // coarse around the whole plausible range, then fine around the winner
        double[][] passes = {{0.5, 12, 6, 6}, {0.1, 0.5, 0.5, 0.5}};
        for (double[] pass : passes) {
            double step = pass[0];
            double[] c = best.clone();
            for (double p = c[0] - pass[1]; p <= c[0] + pass[1] + 1e-9; p += step)
                for (double y = c[1] - pass[2]; y <= c[1] + pass[2] + 1e-9; y += step)
                    for (double r = c[2] - pass[3]; r <= c[2] + pass[3] + 1e-9; r += step) {
                        double cst = cost(rt, shots, new VisionConfig.Mount(p, y, r, cur.xMM, cur.yMM, cur.zMM));
                        if (cst < bestCost) {
                            bestCost = cst;
                            best = new double[]{p, y, r};
                        }
                    }
        }
        return new double[]{best[0], best[1], best[2], bestCost};
    }

    /** The fit must recover a mount we forward-project from, using the production maths both ways. */
    @Test
    public void recoversASyntheticMount() {
        VisionConfig.Camera lens = VisionConfig.ARDUCAM;
        VisionConfig.Mount truth = new VisionConfig.Mount(29.3, 1.2, -0.8,
                lens.mount.xMM, lens.mount.yMM, lens.mount.zMM);
        Matrix r = Raytracer.camToRobot(truth);
        double radius = VisionConfig.POLLEN_DIAMETER_MM / 2.0;

        List<Shot> shots = new ArrayList<>();
        for (double fwd : new double[]{500, 750, 1000, 1250, 1500}) {
            for (double lat : new double[]{-300, 0, 300}) {
                // ball centre in the camera frame: R^T (centre - lens)
                double[] d = {lat - truth.xMM, radius - truth.yMM, fwd - truth.zMM};
                double[] c = new double[3];
                for (int k = 0; k < 3; k++) {
                    for (int i = 0; i < 3; i++) c[k] += r.get(i, k) * d[i];
                }
                double psi = Math.atan2(c[0], c[2]), dx = Math.asin(radius / Math.hypot(c[0], c[2]));
                double th = Math.atan2(c[1], c[2]), dy = Math.asin(radius / Math.hypot(c[1], c[2]));
                Shot s = new Shot();
                s.fwd = fwd;
                s.lat = lat;
                s.radius = radius;
                s.uL = lens.cx + lens.fx * Math.tan(psi - dx);
                s.uR = lens.cx + lens.fx * Math.tan(psi + dx);
                s.vT = lens.cy + lens.fy * Math.tan(th - dy);
                s.vB = lens.cy + lens.fy * Math.tan(th + dy);
                shots.add(s);
            }
        }

        double[] best = fit(new Raytracer(lens), shots, lens.mount);
        assertEquals(truth.pitchDeg, best[0], 0.15);
        assertEquals(truth.yawDeg, best[1], 0.15);
        assertEquals(truth.rollDeg, best[2], 0.15);
    }

    @Test
    public void fitMount() throws Exception {
        InputStream in = getClass().getResourceAsStream(CSV);
        assumeNotNull(in);
        List<Shot> shots = load(in);
        assertTrue("CSV has no usable shots", !shots.isEmpty());

        VisionConfig.Mount cur = VisionConfig.ARDUCAM.mount;
        Raytracer rt = new Raytracer(VisionConfig.ARDUCAM);
        double currentCost = cost(rt, shots, cur);
        double[] best = fit(rt, shots, cur);

        System.out.printf(Locale.US, "RaytracerFitTest: %d shots%n", shots.size());
        System.out.printf(Locale.US, "  current mount  pitch %.2f yaw %.2f roll %.2f  RMS %.1f mm%n",
                cur.pitchDeg, cur.yawDeg, cur.rollDeg, Math.sqrt(currentCost / shots.size() / 2));
        System.out.printf(Locale.US, "  best mount     pitch %.2f yaw %.2f roll %.2f  RMS %.1f mm%n",
                best[0], best[1], best[2], Math.sqrt(best[3] / shots.size() / 2));
        System.out.printf(Locale.US, "  PASTE INTO VisionConfig.ARDUCAM: new Mount(%.2f, %.2f, %.2f, %.3f, %.3f, %.3f)%n",
                best[0], best[1], best[2], cur.xMM, cur.yMM, cur.zMM);
        assertTrue("fit must not be worse than the current mount", best[3] <= currentCost + 1e-9);
    }
}
