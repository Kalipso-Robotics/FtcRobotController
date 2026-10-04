package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;

public class PoseHistoryTest {

    private static final double TOL = 1e-9;

    @Test
    public void emptyHistory_isNull() {
        assertNull(new PoseHistory(8).at(100));
    }

    @Test
    public void interpolatesMidway() {
        PoseHistory h = new PoseHistory(8);
        h.record(1000, new Position(0, 0, 0));
        h.record(2000, new Position(100, -50, 1.0));
        Position p = h.at(1500);
        assertEquals(50, p.getX(), TOL);
        assertEquals(-25, p.getY(), TOL);
        assertEquals(0.5, p.getTheta(), TOL);
    }

    @Test
    public void headingInterpolatesAcrossTheWrap() {
        PoseHistory h = new PoseHistory(8);
        h.record(1000, new Position(0, 0, Math.toRadians(179)));
        h.record(2000, new Position(0, 0, Math.toRadians(-179)));
        double deg = Math.abs(Math.toDegrees(h.at(1500).getTheta()));
        assertEquals(180, deg, 1e-6);
    }

    @Test
    public void olderThanOldest_isNull_newerThanNewest_isNewest() {
        PoseHistory h = new PoseHistory(8);
        h.record(1000, new Position(1, 2, 0));
        h.record(2000, new Position(3, 4, 0));
        assertNull(h.at(999));
        assertEquals(3, h.at(5000).getX(), TOL);
        assertEquals(1, h.at(1000).getX(), TOL);
    }

    @Test
    public void outOfOrderSampleIsIgnored() {
        PoseHistory h = new PoseHistory(8);
        h.record(2000, new Position(3, 0, 0));
        h.record(1500, new Position(99, 0, 0));
        h.record(2000, new Position(98, 0, 0));
        assertEquals(3, h.at(2000).getX(), TOL);
        assertNull(h.at(1500));
    }

    @Test
    public void capacityWrapAround_keepsNewestWindow() {
        PoseHistory h = new PoseHistory(4);
        for (int i = 1; i <= 10; i++) h.record(i * 100L, new Position(i, 0, 0));
        assertNull(h.at(500));                          // samples 1..6 were overwritten
        assertEquals(7, h.at(700).getX(), TOL);
        assertEquals(8.5, h.at(850).getX(), TOL);
        assertEquals(10, h.at(9999).getX(), TOL);
    }

    @Test
    public void recordCopiesThePosition() {
        PoseHistory h = new PoseHistory(4);
        Position p = new Position(1, 1, 0);
        h.record(100, p);
        p.reset(new Position(50, 50, 0));
        assertEquals(1, h.at(100).getX(), TOL);
        assertNotSame(h.at(100), h.at(100));
    }
}
