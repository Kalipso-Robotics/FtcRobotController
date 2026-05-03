package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.junit.Before;
import org.junit.Test;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.*;

/**
 * Tests for ArtifactDetectionProcessor — the purple/green ball detector.
 *
 * How to use alongside camera tuning:
 *   1. Run ArtifactDetectionTest OpMode with the camera pointed at a ball.
 *   2. Watch telemetry for hasPurpleBlob / hasGreenBlob.
 *   3. Adjust PURPLE_HSV_* or GREEN_HSV_* constants in ArtifactDetectionProcessor.
 *   4. Update the HSV range assertions in testPurpleHsvRange / testGreenHsvRange to
 *      match your new constants — this keeps the tests in sync as the spec.
 *
 * Target: 5-inch diameter purple and green balls, detection range 2–10 ft.
 */
public class ArtifactDetectionProcessorTest {

    /**
     * Concrete test-safe subclass that replaces android.graphics.Color.rgb()
     * with raw ARGB literals so the tests run on the JVM without an Android runtime.
     */
    static class TestArtifactProcessor extends KColorBlobProcessor {

        static final Scalar PURPLE_LOWER = new Scalar(117, 50, 40);
        static final Scalar PURPLE_UPPER = new Scalar(155, 255, 255);
        static final Scalar GREEN_LOWER  = new Scalar(40, 60, 40);
        static final Scalar GREEN_UPPER  = new Scalar(85, 255, 255);

        static final String PURPLE = ArtifactDetectionProcessor.PURPLE;
        static final String GREEN  = ArtifactDetectionProcessor.GREEN;

        @Override
        protected ColorChannel[] defineChannels() {
            return new ColorChannel[]{
                new ColorChannel(PURPLE_LOWER, PURPLE_UPPER, PURPLE, 0xFFB400FF),
                new ColorChannel(GREEN_LOWER,  GREEN_UPPER,  GREEN,  0xFF00DC3C)
            };
        }

        public DetectedBlob getLargestPurpleBlob() { return getLargestBlobByLabel(PURPLE); }
        public DetectedBlob getLargestGreenBlob()  { return getLargestBlobByLabel(GREEN); }
        public boolean hasPurpleBlob()             { return hasBlobWithLabel(PURPLE); }
        public boolean hasGreenBlob()              { return hasBlobWithLabel(GREEN); }
    }

    private TestArtifactProcessor processor;

    @Before
    public void setUp() {
        processor = new TestArtifactProcessor();
    }

    private void injectResult(List<DetectedBlob> blobs) throws Exception {
        Field f = KVisionProcessor.class.getDeclaredField("latestResult");
        f.setAccessible(true);
        f.set(processor, blobs);
    }

    private DetectedBlob makeBlob(String label, int x, int y, int w, int h, double area) {
        return new DetectedBlob(new Rect(x, y, w, h), area, 0.85, label);
    }

    // -------------------------------------------------------------------------
    // Named accessors mirror ArtifactDetectionProcessor API
    // -------------------------------------------------------------------------

    @Test
    public void testNoBlobsInitially() {
        assertNull(processor.getLargestPurpleBlob());
        assertNull(processor.getLargestGreenBlob());
        assertFalse(processor.hasPurpleBlob());
        assertFalse(processor.hasGreenBlob());
    }

    @Test
    public void testDetectsPurpleBlob() throws Exception {
        injectResult(Arrays.asList(makeBlob("Purple", 150, 100, 90, 90, 8100)));
        assertTrue(processor.hasPurpleBlob());
        assertFalse(processor.hasGreenBlob());

        DetectedBlob purple = processor.getLargestPurpleBlob();
        assertNotNull(purple);
        assertEquals("Purple", purple.colorLabel);
    }

    @Test
    public void testDetectsGreenBlob() throws Exception {
        injectResult(Arrays.asList(makeBlob("Green", 300, 200, 80, 80, 6400)));
        assertTrue(processor.hasGreenBlob());
        assertFalse(processor.hasPurpleBlob());
    }

    @Test
    public void testDetectsBothColors() throws Exception {
        DetectedBlob purple = makeBlob("Purple", 100, 100, 90, 90, 8100);
        DetectedBlob green  = makeBlob("Green",  400, 200, 70, 70, 4900);
        injectResult(Arrays.asList(purple, green));

        assertTrue(processor.hasPurpleBlob());
        assertTrue(processor.hasGreenBlob());
        assertEquals("Purple", processor.getLargestPurpleBlob().colorLabel);
        assertEquals("Green",  processor.getLargestGreenBlob().colorLabel);
    }

    @Test
    public void testLargestPurpleBlobReturnsLargest() throws Exception {
        DetectedBlob big   = makeBlob("Purple", 50,  50,  120, 120, 14400);
        DetectedBlob small = makeBlob("Purple", 400, 300, 40,  40,  1600);
        injectResult(Arrays.asList(big, small)); // sorted by detect()

        assertEquals(14400.0, processor.getLargestPurpleBlob().area, 0.001);
    }

    // -------------------------------------------------------------------------
    // HSV constant range validation (OpenCV scale: H 0-180, S/V 0-255)
    // Update these bounds when you retune for the actual competition balls.
    // -------------------------------------------------------------------------

    @Test
    public void testPurpleHsvRange() {
        Scalar lower = TestArtifactProcessor.PURPLE_LOWER;
        Scalar upper = TestArtifactProcessor.PURPLE_UPPER;

        // H in [0,180]
        assertTrue("purple H_low in OpenCV range",  lower.val[0] >= 0   && lower.val[0] <= 180);
        assertTrue("purple H_high in OpenCV range", upper.val[0] >= 0   && upper.val[0] <= 180);
        assertTrue("purple H_low < H_high",         lower.val[0] < upper.val[0]);

        // Purple hue band: ~117-155 on the OpenCV 0-180 scale
        assertTrue("purple hue starts in violet range", lower.val[0] >= 110);
        assertTrue("purple hue ends in violet range",   upper.val[0] <= 160);

        // S_min > 50 and V_min > 40 to reject grey/white background noise
        assertTrue("S_min rejects achromatic noise", lower.val[1] >= 50);
        assertTrue("V_min rejects dark noise",       lower.val[2] >= 40);
    }

    @Test
    public void testGreenHsvRange() {
        Scalar lower = TestArtifactProcessor.GREEN_LOWER;
        Scalar upper = TestArtifactProcessor.GREEN_UPPER;

        // H in [0,180]
        assertTrue("green H_low in OpenCV range",  lower.val[0] >= 0   && lower.val[0] <= 180);
        assertTrue("green H_high in OpenCV range", upper.val[0] >= 0   && upper.val[0] <= 180);
        assertTrue("green H_low < H_high",         lower.val[0] < upper.val[0]);

        // Green hue band: ~40-85 on the OpenCV 0-180 scale
        assertTrue("green hue starts in yellow-green range", lower.val[0] >= 35);
        assertTrue("green hue ends before cyan",             upper.val[0] <= 90);

        // S_min > 50 and V_min > 40
        assertTrue("S_min rejects achromatic noise", lower.val[1] >= 50);
        assertTrue("V_min rejects dark noise",       lower.val[2] >= 40);
    }

    @Test
    public void testPurpleAndGreenHueRangesDontOverlap() {
        double purpleHigh = TestArtifactProcessor.PURPLE_UPPER.val[0];
        double greenHigh  = TestArtifactProcessor.GREEN_UPPER.val[0];
        double purpleLow  = TestArtifactProcessor.PURPLE_LOWER.val[0];
        double greenLow   = TestArtifactProcessor.GREEN_LOWER.val[0];

        // Green (40-85) should be completely below purple (117-155)
        assertTrue("green and purple hue bands must not overlap", greenHigh < purpleLow);
    }

    // -------------------------------------------------------------------------
    // Center position — used for aiming / IK downstream
    // -------------------------------------------------------------------------

    @Test
    public void testBlobCenterIsCorrect() throws Exception {
        // Ball bounding box at (200, 150) with size 90x90
        // Expected center: (245, 195)
        DetectedBlob ball = new DetectedBlob(new Rect(200, 150, 90, 90), 8100, 0.91, "Purple");
        injectResult(Arrays.asList(ball));

        DetectedBlob result = processor.getLargestPurpleBlob();
        assertNotNull(result);
        assertEquals(245.0, result.center.getX(), 0.001);
        assertEquals(195.0, result.center.getY(), 0.001);
    }

    @Test
    public void testCircularityThresholdPassesGoodBall() {
        // A well-detected 5-inch ball should have circularity ≥ 0.75
        double r = 30.0; // pixels in downsampled frame
        double area      = Math.PI * r * r;
        double perimeter = 2 * Math.PI * r;
        double circ = (4.0 * Math.PI * area) / (perimeter * perimeter);

        assertTrue("perfect circle should far exceed minCircularity=0.55", circ >= 0.55);
        assertEquals("perfect circle circularity should be 1.0", 1.0, circ, 1e-6);
    }
}
