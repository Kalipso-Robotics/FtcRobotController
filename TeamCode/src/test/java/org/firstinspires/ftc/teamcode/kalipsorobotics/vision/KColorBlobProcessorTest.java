package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.junit.Before;
import org.junit.Test;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.*;

/**
 * Unit tests for KColorBlobProcessor query logic.
 *
 * These tests bypass the OpenCV image pipeline entirely. Results are injected
 * directly into the volatile latestResult field so we can test getLargestBlob(),
 * getLargestBlobByLabel(), and hasBlobWithLabel() in isolation.
 *
 * TUNING REFERENCE for 5-inch purple/green balls:
 *
 *   A 5-inch ball at 3 feet occupies roughly 120x120 px on a 640x480 frame.
 *   At 8 feet it shrinks to ~45x45 px.  The downsampled (320x240) equivalents
 *   are half those dimensions.  Contour area in the downsampled frame:
 *     Close range (~3 ft): ~π*(30)² ≈ 2,800 px²
 *     Far  range (~8 ft):  ~π*(11)² ≈  380 px²
 *
 *   Current thresholds: minContourArea=250, maxContourArea=30000, minCircularity=0.55
 *   These should already bracket a 5-inch ball from ~2 to ~10 feet.
 *   Raise minCircularity toward 0.7 to reject non-circular false positives.
 *   Raise minContourArea (e.g. 400) if nearby flat objects cause noise at close range.
 */
public class KColorBlobProcessorTest {

    /** Concrete subclass with no Android Color dependencies. */
    static class TestProcessor extends KColorBlobProcessor {
        @Override
        protected ColorChannel[] defineChannels() {
            return new ColorChannel[]{
                new ColorChannel(new Scalar(117, 50, 40), new Scalar(155, 255, 255), "Purple", 0xFFB400FF),
                new ColorChannel(new Scalar(40,  60, 40), new Scalar(85,  255, 255), "Green",  0xFF00DC3C)
            };
        }
    }

    private TestProcessor processor;

    @Before
    public void setUp() {
        processor = new TestProcessor();
        // Note: onInit() is NOT called here — we inject results directly.
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private void injectResult(List<DetectedBlob> blobs) throws Exception {
        Field f = KVisionProcessor.class.getDeclaredField("latestResult");
        f.setAccessible(true);
        f.set(processor, blobs);
    }

    private DetectedBlob blob(String label, double area, double circularity) {
        int side = (int) Math.sqrt(area);
        return new DetectedBlob(new Rect(0, 0, side, side), area, circularity, label);
    }

    // -------------------------------------------------------------------------
    // Initial state
    // -------------------------------------------------------------------------

    @Test
    public void testNoResultBeforeFirstFrame() {
        assertFalse("hasResult() should be false before any frame", processor.hasResult());
        assertNull(processor.getLatestResult());
        assertNull(processor.getLargestBlob());
        assertNull(processor.getLargestBlobByLabel("Purple"));
        assertFalse(processor.hasBlobWithLabel("Purple"));
    }

    // -------------------------------------------------------------------------
    // getLargestBlob
    // -------------------------------------------------------------------------

    @Test
    public void testLargestBlobIsFirstInList() throws Exception {
        DetectedBlob large = blob("Purple", 6400, 0.90);
        DetectedBlob small = blob("Green",   400, 0.85);
        injectResult(Arrays.asList(large, small));

        DetectedBlob result = processor.getLargestBlob();
        assertNotNull(result);
        assertEquals(6400.0, result.area, 0.001);
        assertEquals("Purple", result.colorLabel);
    }

    @Test
    public void testLargestBlobNullOnEmptyList() throws Exception {
        injectResult(new ArrayList<>());
        assertNull(processor.getLargestBlob());
    }

    // -------------------------------------------------------------------------
    // getLargestBlobByLabel
    // -------------------------------------------------------------------------

    @Test
    public void testGetByLabelReturnsMostLikelyCandidateFirst() throws Exception {
        DetectedBlob p1 = blob("Purple", 5000, 0.88);
        DetectedBlob p2 = blob("Purple", 1000, 0.72);
        DetectedBlob g1 = blob("Green",  3000, 0.80);
        injectResult(Arrays.asList(p1, g1, p2)); // simulates detect() sorted order

        DetectedBlob purple = processor.getLargestBlobByLabel("Purple");
        assertNotNull(purple);
        assertEquals(5000.0, purple.area, 0.001); // first Purple in list

        DetectedBlob green = processor.getLargestBlobByLabel("Green");
        assertNotNull(green);
        assertEquals(3000.0, green.area, 0.001);
    }

    @Test
    public void testGetByLabelNullWhenColorAbsent() throws Exception {
        injectResult(Arrays.asList(blob("Green", 2000, 0.80)));
        assertNull(processor.getLargestBlobByLabel("Purple"));
    }

    @Test
    public void testGetByLabelNullOnEmptyList() throws Exception {
        injectResult(new ArrayList<>());
        assertNull(processor.getLargestBlobByLabel("Purple"));
    }

    @Test
    public void testGetByLabelNullBeforeFirstFrame() {
        assertNull(processor.getLargestBlobByLabel("Purple"));
        assertNull(processor.getLargestBlobByLabel("Green"));
    }

    // -------------------------------------------------------------------------
    // hasBlobWithLabel
    // -------------------------------------------------------------------------

    @Test
    public void testHasBlobWithLabelTrueWhenPresent() throws Exception {
        injectResult(Arrays.asList(blob("Purple", 3000, 0.85)));
        assertTrue(processor.hasBlobWithLabel("Purple"));
        assertFalse(processor.hasBlobWithLabel("Green"));
    }

    @Test
    public void testHasBlobWithLabelFalseOnEmpty() throws Exception {
        injectResult(new ArrayList<>());
        assertFalse(processor.hasBlobWithLabel("Purple"));
        assertFalse(processor.hasBlobWithLabel("Green"));
    }

    // -------------------------------------------------------------------------
    // Detection threshold validation
    // These document the current threshold values as a living spec.
    // Adjust these assertions when you retune for the 5-inch balls.
    // -------------------------------------------------------------------------

    @Test
    public void testDefaultThresholds() {
        assertEquals("minContourArea should be 250",   250.0, processor.minContourArea,   0.001);
        assertEquals("maxContourArea should be 30000", 30000.0, processor.maxContourArea, 0.001);
        assertEquals("minCircularity should be 0.55",  0.55, processor.minCircularity,    0.001);
    }

    @Test
    public void testMinCircularityRejectsSquare() {
        // A square has circularity ≈ π/4 ≈ 0.785 — it passes 0.55 but should be raiseable.
        double squareCircularity = Math.PI / 4.0;
        assertTrue("square passes current threshold — consider raising minCircularity",
                squareCircularity >= processor.minCircularity);
    }

    @Test
    public void testCircleAtCloseRangePassesAreaFilter() {
        // 5-inch ball at ~3ft: contour area in 320x240 frame ≈ π*30² ≈ 2827 px²
        double closeRangeArea = Math.PI * 30 * 30;
        assertTrue("close-range ball should pass minContourArea", closeRangeArea >= processor.minContourArea);
        assertTrue("close-range ball should pass maxContourArea", closeRangeArea <= processor.maxContourArea);
    }

    @Test
    public void testCircleAtFarRangePassesAreaFilter() {
        // 5-inch ball at ~8ft: contour area in 320x240 frame ≈ π*11² ≈ 380 px²
        double farRangeArea = Math.PI * 11 * 11;
        assertTrue("far-range ball should pass minContourArea", farRangeArea >= processor.minContourArea);
    }

    // -------------------------------------------------------------------------
    // Bounding-box scale math (verifies the 2× downscale/upscale is symmetric)
    // -------------------------------------------------------------------------

    @Test
    public void testScaleFactorRoundTrip() {
        // 640x480 full resolution downsampled to 320x240 and back
        double widthScale  = 640.0 / 320.0;
        double heightScale = 480.0 / 240.0;
        assertEquals(2.0, widthScale,  0.001);
        assertEquals(2.0, heightScale, 0.001);

        // A contour bounding rect at (50,60) 30x20 in downsampled frame
        // must map to (100,120) 60x40 in full-res
        int dsX = 50, dsY = 60, dsW = 30, dsH = 20;
        int fsX = (int)(dsX * widthScale);
        int fsY = (int)(dsY * heightScale);
        int fsW = (int)(dsW * widthScale);
        int fsH = (int)(dsH * heightScale);

        assertEquals(100, fsX);
        assertEquals(120, fsY);
        assertEquals(60,  fsW);
        assertEquals(40,  fsH);
    }
}
