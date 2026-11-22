package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.junit.Test;

/** The aim math in TurretAutoAlign: field point + robot pose -> turret angle (turret + = same way as + heading). */
public class TurretAutoAlignTest {

    private static double aimDeg(Point target, double x, double y, double headingDeg, double offsetDeg) {
        return Math.toDegrees(TurretAutoAlign.calculateTargetAngleRad(
                target, new Position(x, y, Math.toRadians(headingDeg)), offsetDeg));
    }

    @Test
    public void targetStraightAhead_isZero() {
        assertEquals(0, aimDeg(new Point(1000, 0), 0, 0, 0, 0), 1e-9);
    }

    @Test
    public void targetOnPlusYSide_isPlus90() {
        assertEquals(90, aimDeg(new Point(0, 1000), 0, 0, 0, 0), 1e-9);
    }

    @Test
    public void robotTurnedToFaceTarget_isZero() {
        assertEquals(0, aimDeg(new Point(0, 1000), 0, 0, 90, 0), 1e-9);
    }

    @Test
    public void offsetIsAdded() {
        assertEquals(2, aimDeg(new Point(1000, 0), 0, 0, 0, 2), 1e-9);
        assertEquals(92, aimDeg(new Point(0, 1000), 0, 0, 0, 2), 1e-9);
    }

    @Test
    public void resultIsAlwaysWithinPlusMinus180() {
        for (int h = -720; h <= 720; h += 15) {
            for (int a = 0; a < 360; a += 15) {
                Point target = new Point(1000 * Math.cos(Math.toRadians(a)), 1000 * Math.sin(Math.toRadians(a)));
                double deg = aimDeg(target, 0, 0, h, 0);
                assertTrue("h=" + h + " a=" + a + " -> " + deg, deg >= -180 - 1e-9 && deg <= 180 + 1e-9);
            }
        }
    }

    /** RedDecideAuto drives to the launch point facing the Cell (driveTo faceToward), so the turret barely turns. */
    @Test
    public void atLaunchPointFacingCell_isAboutZero() {
        checkFacing(FieldConfig.aLaunchPoint, FieldConfig.aTargetPoint);
        checkFacing(FieldConfig.bLaunchPoint, FieldConfig.bTargetPoint);
    }

    private static void checkFacing(Point launch, Point target) {
        double headingDeg = Math.toDegrees(Math.atan2(target.getY() - launch.getY(), target.getX() - launch.getX()));
        assertEquals(0, aimDeg(target, launch.getX(), launch.getY(), headingDeg, 0), 1e-9);
        // 10 deg heading error at the end of the drive -> turret corrects by -10 deg.
        assertEquals(-10, aimDeg(target, launch.getX(), launch.getY(), headingDeg + 10, 0), 1e-9);
    }

    /** Finding 3: a target just behind flips from +179 to -179 as it crosses straight back (a full-turn swing request). */
    @Test
    public void targetBehind_flipsSignAcrossStraightBack() {
        double left = aimDeg(new Point(-1000, 17.45), 0, 0, 0, 0);   // ~1 deg to the +y side of straight back
        double right = aimDeg(new Point(-1000, -17.45), 0, 0, 0, 0); // ~1 deg to the -y side
        assertEquals(179, left, 0.01);
        assertEquals(-179, right, 0.01);
    }
}
