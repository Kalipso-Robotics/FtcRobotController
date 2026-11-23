package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

import java.util.List;

/**
 * Turns ONE camera frame of HIVE tags into two answers:
 *   1. the goal state: which of our CELLs is up (A or B), or "tipping" when the HIVE is swinging or the tags disagree;
 *   2. the robot's field x/y (heading stays from the IMU).
 *
 * How (picture the camera looking up at the four tags under a CELL):
 *   - The camera says where each tag is in front of it. Tilting that by the camera pitch gives the tag's HEIGHT.
 *   - A tag sits ~360 mm higher when its CELL is up than when it is down, so the height says "up" or "down".
 *   - Each tag votes: an A tag that is up, or a B tag that is down, means "A is up". Majority (at least 2, none
 *     against) wins.
 *   - Knowing where that tag really is on the field, we walk back from the tag through camera -> turret -> robot to
 *     get one robot x/y per tag, then average them.
 *
 * Pure Java math, no hardware, so HiveTagLocalizerTest checks it on the laptop.
 * The sliding window over several frames lives in AprilTagDetectionAction.
 */
public final class HiveTagLocalizer {

    private HiveTagLocalizer() {
    }

    /** Our two CELLs. A = the high-y CELL of our own frame, B = the low-y one (see FieldConfig). */
    public enum Cell {
        A, // high-y CELL
        B  // low-y CELL
    }

    /** What one frame says. */
    public static final class Fix {
        public final Point robot;     // robot field x/y in mm, null = the tags gave no trustworthy position
        public final Cell upCell;     // which of our CELLs is up, null = not sure (too few tags, or tipping)
        public final boolean tipping; // true = a tag was between its up and down heights, or the tags disagreed
        public final int tagsUsed;    // tags that were ours and gave a clear up/down height

        public Fix(Point robot, Cell upCell, boolean tipping, int tagsUsed) {
            this.robot = robot;
            this.upCell = upCell;
            this.tipping = tipping;
            this.tagsUsed = tagsUsed;
        }
    }

    private static final double FIELD_MM = 144 * 25.4;
    private static final int MIN_VOTES_FOR_STATE = 2;

    /**
     * @param observations   every tag in this frame (camera frame, mm)
     * @param headingRad     robot heading from odometry (IMU)
     * @param turretAngleRad turret angle (commanded, there is no encoder)
     */
    public static Fix solve(List<TagObservation> observations, double headingRad, double turretAngleRad, AllianceColor allianceColor) {
        double pitch = Math.toRadians(AprilTagConfig.CAM_PITCH_DEG);

        // Where the camera is in the robot frame (x forward, y right). The turret angle turns the camera the same way it
        // aims (+ = clockwise, like heading). TODO-verify on the robot: turn the turret and check x/y holds still.
        Position camInRobot = AprilTagConfig.TURRET_REL_CAM_POS.toNewFrame(
                TurretConfig.ROBOT_REL_TURRET_POINT.getX(), TurretConfig.ROBOT_REL_TURRET_POINT.getY(), turretAngleRad);

        int votesAUp = 0;
        int votesBUp = 0;
        boolean someTagBetweenBands = false;

        // Robot x/y from each tag, kept so we can check they agree.
        double[] robotXs = new double[observations.size()];
        double[] robotYs = new double[observations.size()];
        int positions = 0;

        for (TagObservation tag : observations) {
            int id = tag.getTagId();

            // 1. Only our own CELLs' tags (backup for the Limelight's ID filter).
            char tagCell = AprilTagConfig.ownCellOf(id, allianceColor);
            if (tagCell == 0) continue;

            // 2. Camera frame (OpenCV: x right, y down, z out of the lens) -> level frame. The camera is pitched up.
            double camRight = tag.getTagRelCamXMM();
            double camDown = tag.getTagRelCamYMM();
            double camOut = tag.getTagRelCamZMM();
            double tagUp = camOut * Math.sin(pitch) - camDown * Math.cos(pitch);      // above the lens
            double tagForward = camOut * Math.cos(pitch) + camDown * Math.sin(pitch); // flat, along the camera's aim
            double tagRight = camRight;                                               // flat, to the camera's right
            double tagHeight = AprilTagConfig.CAM_HEIGHT_MM + tagUp;

            // 3. Up or down? Compare with where this tag would be in each case.
            double[] spotIfUp = AprilTagConfig.tagFieldXYZ(id, true, allianceColor);
            double[] spotIfDown = AprilTagConfig.tagFieldXYZ(id, false, allianceColor);
            boolean looksUp = Math.abs(tagHeight - spotIfUp[2]) <= AprilTagConfig.TAG_HEIGHT_TOLERANCE_MM;
            boolean looksDown = Math.abs(tagHeight - spotIfDown[2]) <= AprilTagConfig.TAG_HEIGHT_TOLERANCE_MM;
            if (looksUp == looksDown) { // in neither band (or, with a huge tolerance, both): HIVE is moving
                someTagBetweenBands = true;
                continue;
            }

            // 4. Vote. "My A tag is up" and "my B tag is down" both mean A is up.
            boolean saysAUp = (tagCell == 'A') == looksUp;
            if (saysAUp) votesAUp++;
            else votesBUp++;

            // 5. Robot position from this tag: tag's field spot minus (robot -> tag), turned by the robot heading.
            double[] tagSpot = looksUp ? spotIfUp : spotIfDown;
            Position tagInRobot = new Position(tagForward, tagRight, 0).toNewFrame(camInRobot);
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);
            robotXs[positions] = tagSpot[0] - (tagInRobot.getX() * cos - tagInRobot.getY() * sin);
            robotYs[positions] = tagSpot[1] - (tagInRobot.getX() * sin + tagInRobot.getY() * cos);
            positions++;
        }

        // Goal state: at least 2 votes on one side and none on the other.
        boolean votesSplit = votesAUp > 0 && votesBUp > 0;
        boolean tipping = someTagBetweenBands || votesSplit;
        Cell upCell = null;
        if (votesAUp >= MIN_VOTES_FOR_STATE && votesBUp == 0) upCell = Cell.A;
        if (votesBUp >= MIN_VOTES_FOR_STATE && votesAUp == 0) upCell = Cell.B;

        return new Fix(averageIfTagsAgree(robotXs, robotYs, positions), upCell, tipping, positions);
    }

    /** Average robot spot, or null if any tag is further than MAX_TAG_SPREAD_MM from it or it is off the field. */
    private static Point averageIfTagsAgree(double[] xs, double[] ys, int count) {
        if (count == 0) return null;
        double sumX = 0;
        double sumY = 0;
        for (int i = 0; i < count; i++) {
            sumX += xs[i];
            sumY += ys[i];
        }
        double averageX = sumX / count;
        double averageY = sumY / count;

        for (int i = 0; i < count; i++) {
            boolean tooFarFromAverage = Math.hypot(xs[i] - averageX, ys[i] - averageY) > AprilTagConfig.MAX_TAG_SPREAD_MM;
            if (tooFarFromAverage) return null;
        }
        boolean onField = averageX >= 0 && averageX <= FIELD_MM && averageY >= 0 && averageY <= FIELD_MM;
        return onField ? new Point(averageX, averageY) : null;
    }
}
