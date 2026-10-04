package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Replays tape-measured ground truth through the production ball path
 * (Raytracer.estimate(VisionRecognition, growPx): edge growth and clip gate included).
 *
 * The data is from a 640x480 colour-blob run with the lens as it was then (LEGACY_640 below, kept
 * only so this evidence survives; production is 1280x720 now). The 1280x720 equivalent is
 * RaytracerFitTest on a RaytracingGroundTruth CSV.
 *
 * Rows are per-distance medians of the 30-frame bursts in RaytracingGroundTruth_2026_09_08
 * (17:43 and 17:53 sessions), green 127mm artifact, centred (true lateral 0), distance
 * from robot centre. Bbox is rebuilt as left/right = centreX -/+ W/2, top = bottomY - H.
 *
 * Lateral is NOT asserted: every method reads +45..+75mm left at true 0, the signature of
 * the known-bad fx (444 vs fy 532). Square pixels (fx = fy) cut it to 21mm RMS in the
 * A/B, but fx also feeds AprilTag pose, so it waits for a checkerboard calibration.
 */
public class RaytracerGroundTruthTest {

    /** The 640x480 lens and 28 deg mount this data was fitted against. */
    private static VisionConfig.Camera legacy640(double pitchDeg) {
        return new VisionConfig.Camera("legacy640", 640, 480,
                444.14195, 532.27560, 350.01860, 212.43984,
                0.045011, -0.059862, 0.000330, 0.001499, 0.005590,
                new VisionConfig.Mount(pitchDeg, 0, 0, -157.548, 236.163, 163.470));
    }

    /** Half the colour mask's 19.29 px shrink at 640x480. */
    private static final double GROW_PX = 9.645;

    // {knownFwdMM, bottomX, bottomY, bboxW, bboxH}
    private static final double[][] ROWS = {
            {500, 108, 270, 180, 160}, {600, 155, 208, 142, 130}, {700, 183, 164, 114, 108},
            {800, 204, 134, 96, 92}, {900, 220, 108, 84, 78}, {1000, 232, 88, 72, 68},
            {508, 108, 262, 184, 154}, {609.6, 155, 216, 146, 140}, {711.2, 183, 172, 118, 116},
            {812.8, 202.5, 140, 99, 98}, {914.4, 219, 108, 86, 80}, {1016, 233, 88, 74, 68},
            {1117.6, 243, 74, 66, 62}, {1219.2, 251, 56, 58, 48},
            // bbox top on row 0: clipped by the frame, must be rejected
            {1320.8, 258, 44, 50, 44}, {1422.4, 264, 38, 48, 38},
    };

    private static VisionRecognition blob(double[] r) {
        return new VisionRecognition("Green", 1f, (float) (r[1] - r[3] / 2), (float) (r[2] - r[4]),
                (float) (r[1] + r[3] / 2), (float) r[2], 127.0);
    }

    /** Plan pass criterion: max(5%, 50mm). */
    private static double tol(double known) {
        return Math.max(0.05 * known, 50);
    }

    @Test
    public void ballRay_withinToleranceAtEveryDistance() {
        double se = 0;
        int n = 0;
        for (double[] r : ROWS) {
            Raytracer.Estimate e = new Raytracer(legacy640(28)).estimate(blob(r), GROW_PX);
            if (r[0] > 1300) {
                assertNull("clipped bbox at " + r[0] + " must be rejected", e);
                continue;
            }
            assertNotNull("at " + r[0], e);
            double err = e.robotPos.getY() - r[0];
            assertEquals("forward at " + r[0], r[0], e.robotPos.getY(), tol(r[0]));
            se += err * err;
            n++;
        }
        double rms = Math.sqrt(se / n);
        assertTrue("ball-ray forward RMS " + rms, rms < 20); // 14.3 at 28 deg; 55 at the old 29
    }

    @Test
    public void pitchIsAtTheMinimum() {
        // A 0.25 deg nudge either way must score worse: the fitted pitch sits at the minimum.
        double at = rms(28), lo = rms(27.75), hi = rms(28.25);
        assertTrue(at + " vs " + lo + " / " + hi, at < lo && at < hi);
    }

    private static double rms(double pitchDeg) {
        Raytracer rt = new Raytracer(legacy640(pitchDeg));
        double se = 0;
        int n = 0;
        for (double[] r : ROWS) {
            Raytracer.Estimate e = rt.estimate(blob(r), GROW_PX);
            if (e == null) continue;
            se += Math.pow(e.robotPos.getY() - r[0], 2);
            n++;
        }
        return Math.sqrt(se / n);
    }
}
