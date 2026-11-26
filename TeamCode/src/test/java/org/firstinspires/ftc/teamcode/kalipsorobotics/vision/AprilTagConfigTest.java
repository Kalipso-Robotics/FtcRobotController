package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.junit.Test;

public class AprilTagConfigTest {
    private static final double FIELD = 142 * 25.4;

    @Test
    public void idsPerAlliance() {
        assertArrayEquals(new int[]{34, 35, 36, 37}, AprilTagConfig.cellATagIds(AllianceColor.RED));
        assertArrayEquals(new int[]{30, 31, 32, 33}, AprilTagConfig.cellBTagIds(AllianceColor.RED));
        assertArrayEquals(new int[]{42, 43, 44, 45}, AprilTagConfig.cellATagIds(AllianceColor.BLUE));
        assertArrayEquals(new int[]{38, 39, 40, 41}, AprilTagConfig.cellBTagIds(AllianceColor.BLUE));
        assertEquals('A', AprilTagConfig.ownCellOf(36, AllianceColor.RED));
        assertEquals(0, AprilTagConfig.ownCellOf(36, AllianceColor.BLUE));
    }

    /** Each alliance frame is the other rotated 180 deg, so a Cell's tags have the same numbers for both. */
    @Test
    public void ownCellsHaveTheSameCoordinatesForBothAlliances() {
        for (boolean up : new boolean[]{true, false}) {
            for (int i = 0; i < 4; i++) {
                assertArrayEquals(AprilTagConfig.tagFieldXYZ(AprilTagConfig.cellATagIds(AllianceColor.RED)[i], up, AllianceColor.RED),
                        AprilTagConfig.tagFieldXYZ(AprilTagConfig.cellATagIds(AllianceColor.BLUE)[i], up, AllianceColor.BLUE), 1e-9);
                assertArrayEquals(AprilTagConfig.tagFieldXYZ(AprilTagConfig.cellBTagIds(AllianceColor.RED)[i], up, AllianceColor.RED),
                        AprilTagConfig.tagFieldXYZ(AprilTagConfig.cellBTagIds(AllianceColor.BLUE)[i], up, AllianceColor.BLUE), 1e-9);
            }
        }
    }

    /** Cell A tags on the high-y side, B on the low-y side, both near our Cell target points (FieldConfig). */
    @Test
    public void cellsSitOnTheirOwnEndOfTheHive() {
        double[] a = AprilTagConfig.tagFieldXYZ(34, true, AllianceColor.RED);
        double[] b = AprilTagConfig.tagFieldXYZ(30, true, AllianceColor.RED);
        assertTrue(a[1] > FIELD / 2 && b[1] < FIELD / 2);
        assertEquals(ShooterConfig.TARGET_POINT_1.getX(), a[0], 250);
        assertEquals(ShooterConfig.TARGET_POINT_2.getX(), b[0], 250);
    }

    /** Figure 9-17 order: red far reads 33 32 31 30 left to right, red audience 34 35 36 37. */
    @Test
    public void tagOrderMatchesTheManual() {
        assertTrue(AprilTagConfig.tagFieldXYZ(30, true, AllianceColor.RED)[0] > AprilTagConfig.tagFieldXYZ(33, true, AllianceColor.RED)[0]);
        assertTrue(AprilTagConfig.tagFieldXYZ(37, true, AllianceColor.RED)[0] > AprilTagConfig.tagFieldXYZ(34, true, AllianceColor.RED)[0]);
        // Blue Hive is right of red in the manual's view.
        assertTrue(AprilTagConfig.tagFieldXYZ(42, true, AllianceColor.RED)[0] > AprilTagConfig.tagFieldXYZ(30, true, AllianceColor.RED)[0]);
    }

    /** Tag centres from the official BIOBUZZ field CAD (2026-11-26), Red frame, shifted by (FIELD - 3585.7) / 2 for our field size. */
    @Test
    public void tagCentresMatchFieldCad() {
        double d = (FIELD - 3585.7) / 2;
        double[] xs = {1304.1 + d, 1399.4 + d, 1539.1 + d, 1634.3 + d};
        for (int i = 0; i < 4; i++) {
            double[] up = AprilTagConfig.tagFieldXYZ(34 + i, true, AllianceColor.RED);
            double[] down = AprilTagConfig.tagFieldXYZ(34 + i, false, AllianceColor.RED);
            assertArrayEquals(new double[]{xs[i], 2125.8 + d, 1264.6}, up, 0.2);
            assertArrayEquals(new double[]{xs[i], 2087.7 + d, 902.2}, down, 0.2);
        }
        assertArrayEquals(new double[]{1634.3 + d, 1460.0 + d, 1264.6}, AprilTagConfig.tagFieldXYZ(30, true, AllianceColor.RED), 0.2);
        assertArrayEquals(new double[]{2281.6 + d, 1498.0 + d, 902.2}, AprilTagConfig.tagFieldXYZ(42, false, AllianceColor.RED), 0.2);
    }

    @Test
    public void upAndDownHeightsAreSeparable() {
        double up = AprilTagConfig.tagFieldXYZ(34, true, AllianceColor.RED)[2];
        double down = AprilTagConfig.tagFieldXYZ(34, false, AllianceColor.RED)[2];
        assertTrue(up - down > 2 * AprilTagConfig.TAG_HEIGHT_TOLERANCE_MM);
    }

    @Test
    public void nonHiveIdsHaveNoPose() {
        assertNull(AprilTagConfig.tagFieldXYZ(24, true, AllianceColor.RED));
        assertNull(AprilTagConfig.tagFieldXYZ(46, true, AllianceColor.RED));
    }
}
