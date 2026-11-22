package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig.*;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class HiveTagLocalizerTest {

    /** What the Limelight would report (OpenCV camera frame) for a robot at (x, y, heading) with the turret at turretRad. */
    static TagObservation see(int id, boolean cellUp, AllianceColor alliance, double x, double y, double heading, double turretRad) {
        double ct = Math.cos(turretRad), st = Math.sin(turretRad);
        double camRx = TurretConfig.ROBOT_REL_TURRET_POINT.getX() + ct * TURRET_REL_CAM_POS.getX() - st * TURRET_REL_CAM_POS.getY();
        double camRy = TurretConfig.ROBOT_REL_TURRET_POINT.getY() + st * TURRET_REL_CAM_POS.getX() + ct * TURRET_REL_CAM_POS.getY();
        double camX = x + Math.cos(heading) * camRx - Math.sin(heading) * camRy;
        double camY = y + Math.sin(heading) * camRx + Math.cos(heading) * camRy;

        double[] tag = tagFieldXYZ(id, cellUp, alliance);
        double dx = tag[0] - camX, dy = tag[1] - camY, dz = tag[2] - CAM_HEIGHT_MM;
        double yaw = heading + turretRad + TURRET_REL_CAM_POS.getTheta();
        double levelFwd = dx * Math.cos(yaw) + dy * Math.sin(yaw);
        double right = -dx * Math.sin(yaw) + dy * Math.cos(yaw);
        double p = Math.toRadians(CAM_PITCH_DEG);
        double fwd = levelFwd * Math.cos(p) + dz * Math.sin(p);
        double upCam = -levelFwd * Math.sin(p) + dz * Math.cos(p);
        return new TagObservation(id, new Position(0, 0, 0), 0, right, -upCam, fwd);
    }

    private static List<TagObservation> cluster(int[] ids, boolean up, AllianceColor a, double x, double y, double h, double t) {
        List<TagObservation> list = new ArrayList<>();
        for (int id : ids) list.add(see(id, up, a, x, y, h, t));
        return list;
    }

    @Test
    public void ownCellUp_recoversRobotAndGoalState() {
        double x = 1400, y = 3100, h = Math.toRadians(-80), t = Math.toRadians(-15);
        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(
                cluster(cellATagIds(AllianceColor.RED), true, AllianceColor.RED, x, y, h, t), h, t, AllianceColor.RED);
        assertEquals(x, fix.robot.getX(), 1e-6);
        assertEquals(y, fix.robot.getY(), 1e-6);
        assertEquals(4, fix.tagsUsed);
        assertEquals(HiveTagLocalizer.Cell.A, fix.upCell);
    }

    @Test
    public void ownCellDown_meansTheOtherCellIsUp() {
        double x = 1600, y = 2700, h = Math.toRadians(-100), t = 0.2;
        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(
                cluster(cellATagIds(AllianceColor.RED), false, AllianceColor.RED, x, y, h, t), h, t, AllianceColor.RED);
        assertEquals(x, fix.robot.getX(), 1e-6);
        assertEquals(y, fix.robot.getY(), 1e-6);
        assertEquals(HiveTagLocalizer.Cell.B, fix.upCell);
    }

    @Test
    public void blue_opponentTagsLocalizeButSayNothingAboutOurGoal() {
        double x = 2300, y = 700, h = Math.toRadians(95), t = -0.4;
        // Red's far CELL (30-33) seen from the blue frame.
        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(
                cluster(new int[]{30, 31, 32, 33}, true, AllianceColor.BLUE, x, y, h, t), h, t, AllianceColor.BLUE);
        assertEquals(x, fix.robot.getX(), 1e-6);
        assertEquals(y, fix.robot.getY(), 1e-6);
        assertNull(fix.upCell);
    }

    @Test
    public void tagHalfwayThroughATip_isIgnored() {
        double x = 1500, y = 3000, h = Math.toRadians(-90);
        TagObservation o = see(34, true, AllianceColor.RED, x, y, h, 0);
        double midDrop = (tagFieldXYZ(34, true, AllianceColor.RED)[2] - tagFieldXYZ(34, false, AllianceColor.RED)[2]) / 2;
        // Same view but the tag sits halfway between its up and down heights: shift it straight down in the camera frame.
        double p = Math.toRadians(CAM_PITCH_DEG);
        TagObservation tipping = new TagObservation(34, new Position(0, 0, 0), 0, o.getTagRelCamXMM(),
                o.getTagRelCamYMM() + midDrop * Math.cos(p), o.getTagRelCamZMM() - midDrop * Math.sin(p));
        List<TagObservation> list = new ArrayList<>();
        list.add(tipping);
        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(list, h, 0, AllianceColor.RED);
        assertNull(fix.robot);
        assertNull(fix.upCell);
    }

    @Test
    public void tagsThatDisagree_giveNoPosition() {
        double x = 1500, y = 3000, h = Math.toRadians(-90);
        List<TagObservation> list = cluster(new int[]{34, 35}, true, AllianceColor.RED, x, y, h, 0);
        TagObservation off = list.get(1);
        list.set(1, new TagObservation(35, new Position(0, 0, 0), 0, off.getTagRelCamXMM() + 400, off.getTagRelCamYMM(), off.getTagRelCamZMM()));
        assertNull(HiveTagLocalizer.solve(list, h, 0, AllianceColor.RED).robot);
    }

    /**
     * Hand-built, not from see(): heading 0, turret 0, a tag 1000 mm ahead (level) and 100 mm to the camera's right
     * (OpenCV +x). Field +y is the robot's right at heading 0, so the camera is 1000 mm behind and 100 mm left of the tag.
     */
    @Test
    public void signConventions_handComputed() {
        double[] tag = tagFieldXYZ(34, true, AllianceColor.RED);
        double p = Math.toRadians(CAM_PITCH_DEG), levelFwd = 1000, dz = tag[2] - CAM_HEIGHT_MM;
        double fwd = levelFwd * Math.cos(p) + dz * Math.sin(p);
        double upCam = -levelFwd * Math.sin(p) + dz * Math.cos(p);
        List<TagObservation> list = new ArrayList<>();
        list.add(new TagObservation(34, new Position(0, 0, 0), 0, 100, -upCam, fwd));
        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(list, 0, 0, AllianceColor.RED);
        assertEquals(tag[0] - 1000 - TurretConfig.ROBOT_REL_TURRET_POINT.getX() - TURRET_REL_CAM_POS.getX(), fix.robot.getX(), 1e-6);
        assertEquals(tag[1] - 100 - TurretConfig.ROBOT_REL_TURRET_POINT.getY() - TURRET_REL_CAM_POS.getY(), fix.robot.getY(), 1e-6);
    }

    @Test
    public void nonHiveTags_areIgnored() {
        List<TagObservation> list = new ArrayList<>();
        list.add(new TagObservation(24, new Position(0, 0, 0), 0, 0, -500, 2000));
        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(list, 0, 0, AllianceColor.RED);
        assertNull(fix.robot);
        assertEquals(0, fix.tagsUsed);
    }
}
