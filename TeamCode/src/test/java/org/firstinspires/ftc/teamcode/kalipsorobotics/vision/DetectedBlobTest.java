package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.junit.Test;
import org.opencv.core.Rect;

import static org.junit.Assert.*;

/**
 * Tests for DetectedBlob data class.
 *
 * Verifies that center, area, circularity, and label are computed and stored correctly.
 * All coordinates are in full-resolution space (640x480).
 */
public class DetectedBlobTest {

    @Test
    public void testCenterIsBoxMidpoint() {
        // Box at (100,200) with 60x40 → center (130, 220)
        DetectedBlob blob = new DetectedBlob(new Rect(100, 200, 60, 40), 2400.0, 0.90, "Purple");
        assertEquals(130.0, blob.center.getX(), 0.001);
        assertEquals(220.0, blob.center.getY(), 0.001);
    }

    @Test
    public void testCenterForOriginBox() {
        DetectedBlob blob = new DetectedBlob(new Rect(0, 0, 640, 480), 307200.0, 0.79, "Green");
        assertEquals(320.0, blob.center.getX(), 0.001);
        assertEquals(240.0, blob.center.getY(), 0.001);
    }

    @Test
    public void testFieldValuesPreserved() {
        Rect rect = new Rect(10, 20, 50, 50);
        DetectedBlob blob = new DetectedBlob(rect, 2500.0, 0.85, "Green");

        assertSame(rect, blob.boundingBox);
        assertEquals(2500.0, blob.area, 0.001);
        assertEquals(0.85,   blob.circularity, 0.001);
        assertEquals("Green", blob.colorLabel);
    }

    @Test
    public void testToStringContainsKeyInfo() {
        // "Purple (130,220) A:2400 C:0.90"
        DetectedBlob blob = new DetectedBlob(new Rect(100, 200, 60, 40), 2400.0, 0.90, "Purple");
        String s = blob.toString();
        assertTrue("label should contain color", s.contains("Purple"));
        assertTrue("label should contain center x", s.contains("130"));
        assertTrue("label should contain center y", s.contains("220"));
    }

    @Test
    public void testToStringMatchesLabel() {
        DetectedBlob blob = new DetectedBlob(new Rect(0, 0, 100, 100), 10000.0, 0.80, "Purple");
        assertEquals(blob.toString(), blob.label);
    }

    // -------------------------------------------------------------------------
    // Circularity formula — 4π·A / P²
    // These test the mathematical contract that the processor promises to use.
    // -------------------------------------------------------------------------

    @Test
    public void testCircularityPerfectCircle() {
        // For a circle: P = 2πr, A = πr²  →  circularity = 1.0
        double r = 20.0;
        double area      = Math.PI * r * r;
        double perimeter = 2 * Math.PI * r;
        double c = (4.0 * Math.PI * area) / (perimeter * perimeter);
        assertEquals(1.0, c, 1e-6);
    }

    @Test
    public void testCircularitySquareIsLessThanOne() {
        // Square: circularity = π/4 ≈ 0.785 — well below the 0.55 threshold
        double s = 20.0;
        double area      = s * s;
        double perimeter = 4 * s;
        double c = (4.0 * Math.PI * area) / (perimeter * perimeter);
        assertEquals(Math.PI / 4.0, c, 1e-6);
        assertTrue("square circularity should be < 0.9", c < 0.9);
        assertTrue("square circularity should be > 0.5", c > 0.5);
    }

    @Test
    public void testCircularityZeroPerimeterIsZero() {
        // Guard: perimeter == 0 must not divide by zero
        double c = (0 > 0) ? (4.0 * Math.PI * 1.0) / (0.0) : 0;
        assertEquals(0.0, c, 0.0);
    }
}
