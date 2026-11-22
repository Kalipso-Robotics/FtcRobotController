package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Checks HiveTagLocalizer with made-up camera sightings.
 * Idea: put the robot somewhere we choose, put the HIVE tags where AprilTagConfig says they are, and work out what the
 * camera WOULD see. Then hand that to solve() and check it gives back the robot spot and the right up Cell.
 */
public class HiveTagLocalizerTest {

    // Robot pose used by most tests: somewhere on the field, turned a bit, turret turned a bit.
    private static final double ROBOT_X = 1200;
    private static final double ROBOT_Y = 2600;
    private static final double HEADING = 0.3;
    private static final double TURRET = 0.2;

    /** What the camera would report for tag `id` sitting at field spot `tag` ({x, y, z}). */
    private static TagObservation see(int id, double[] tag, double robotX, double robotY, double heading, double turretAngle) {
        // Where the camera is on the field (same chain as the robot: robot -> turret -> camera).
        Position camInRobot = AprilTagConfig.TURRET_REL_CAM_POS.toNewFrame(
                TurretConfig.ROBOT_REL_TURRET_POINT.getX(), TurretConfig.ROBOT_REL_TURRET_POINT.getY(), -turretAngle);
        Position cam = camInRobot.toNewFrame(robotX, robotY, heading);

        // The tag seen from the camera, flat: how far forward and how far right, plus how far up.
        double dx = tag[0] - cam.getX();
        double dy = tag[1] - cam.getY();
        double forward = dx * Math.cos(cam.getTheta()) + dy * Math.sin(cam.getTheta());
        double right = -dx * Math.sin(cam.getTheta()) + dy * Math.cos(cam.getTheta());
        double up = tag[2] - AprilTagConfig.CAM_HEIGHT_MM;

        // Tilt into the camera's own frame (OpenCV: x right, y down, z out of the lens), camera pitched up.
        double pitch = Math.toRadians(AprilTagConfig.CAM_PITCH_DEG);
        double camX = right;
        double camY = forward * Math.sin(pitch) - up * Math.cos(pitch);
        double camZ = forward * Math.cos(pitch) + up * Math.sin(pitch);
        return new TagObservation(id, new Position(0, 0, 0), 0, camX, camY, camZ);
    }

    private static TagObservation see(int id, boolean cellUp, AllianceColor color) {
        return see(id, AprilTagConfig.tagFieldXYZ(id, cellUp, color), ROBOT_X, ROBOT_Y, HEADING, TURRET);
    }

    @Test
    public void redCellAUp_givesCellAAndRobotSpot() {
        List<TagObservation> seen = new ArrayList<>();
        for (int id = 34; id <= 37; id++) seen.add(see(id, true, AllianceColor.RED));   // Cell A tags, A is up
        for (int id = 30; id <= 31; id++) seen.add(see(id, false, AllianceColor.RED));  // Cell B tags, B is down

        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(seen, HEADING, TURRET, AllianceColor.RED);

        assertEquals(HiveTagLocalizer.Cell.A, fix.upCell);
        assertFalse(fix.tipping);
        assertEquals(6, fix.tagsUsed);
        assertNotNull(fix.robot);
        assertEquals(ROBOT_X, fix.robot.getX(), 1);
        assertEquals(ROBOT_Y, fix.robot.getY(), 1);
    }

    @Test
    public void blueCellBUp_givesCellBAndRobotSpot() {
        List<TagObservation> seen = new ArrayList<>();
        for (int id = 38; id <= 41; id++) seen.add(see(id, true, AllianceColor.BLUE)); // Blue Cell B tags, B is up

        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(seen, HEADING, TURRET, AllianceColor.BLUE);

        assertEquals(HiveTagLocalizer.Cell.B, fix.upCell);
        assertFalse(fix.tipping);
        assertNotNull(fix.robot);
        assertEquals(ROBOT_X, fix.robot.getX(), 1);
        assertEquals(ROBOT_Y, fix.robot.getY(), 1);
    }

    @Test
    public void oneTag_givesNoGoalState() {
        List<TagObservation> seen = new ArrayList<>();
        seen.add(see(34, true, AllianceColor.RED));

        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(seen, HEADING, TURRET, AllianceColor.RED);

        assertNull(fix.upCell);
        assertFalse(fix.tipping);
    }

    @Test
    public void mixedVotes_meansTipping() {
        List<TagObservation> seen = new ArrayList<>();
        seen.add(see(34, true, AllianceColor.RED));  // says A up
        seen.add(see(35, true, AllianceColor.RED));  // says A up
        seen.add(see(36, false, AllianceColor.RED)); // says B up

        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(seen, HEADING, TURRET, AllianceColor.RED);

        assertNull(fix.upCell);
        assertTrue(fix.tipping);
    }

    @Test
    public void tagHalfwayBetweenUpAndDown_meansTipping() {
        double[] up = AprilTagConfig.tagFieldXYZ(34, true, AllianceColor.RED);
        double[] down = AprilTagConfig.tagFieldXYZ(34, false, AllianceColor.RED);
        double[] halfway = {(up[0] + down[0]) / 2, (up[1] + down[1]) / 2, (up[2] + down[2]) / 2};
        List<TagObservation> seen = new ArrayList<>();
        seen.add(see(34, halfway, ROBOT_X, ROBOT_Y, HEADING, TURRET));
        seen.add(see(35, true, AllianceColor.RED));
        seen.add(see(36, true, AllianceColor.RED));

        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(seen, HEADING, TURRET, AllianceColor.RED);

        assertTrue(fix.tipping);
        assertEquals(2, fix.tagsUsed); // the halfway tag gives no vote and no position
    }

    @Test
    public void otherAllianceTags_areIgnored() {
        List<TagObservation> seen = new ArrayList<>();
        seen.add(see(42, true, AllianceColor.BLUE)); // Blue tags while we are Red
        seen.add(see(43, true, AllianceColor.BLUE));

        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(seen, HEADING, TURRET, AllianceColor.RED);

        assertEquals(0, fix.tagsUsed);
        assertNull(fix.upCell);
        assertNull(fix.robot);
    }

    @Test
    public void oneTagFarOff_killsThePosition() {
        List<TagObservation> seen = new ArrayList<>();
        seen.add(see(34, true, AllianceColor.RED));
        seen.add(see(35, true, AllianceColor.RED));
        // Tag 36 as if the robot were 500 mm further along x: a bad solve.
        seen.add(see(36, AprilTagConfig.tagFieldXYZ(36, true, AllianceColor.RED), ROBOT_X + 500, ROBOT_Y, HEADING, TURRET));

        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(seen, HEADING, TURRET, AllianceColor.RED);

        assertNull(fix.robot);
        assertEquals(HiveTagLocalizer.Cell.A, fix.upCell); // the goal state does not need a good position
    }
}
