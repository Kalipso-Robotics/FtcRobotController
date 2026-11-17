package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagFieldLayout;

@Config
public class AprilTagConfig {


    // DECODE goal point. Only Shooter.TARGET_POINT (used by the old decode/auto OpModes) still reads these two.
    public static double APRILTAG_X_REL_FIELD_MM = 3076.6; // 3115 FROM TUNE | 3,076.6 FROM MEASURE FIELD WITH TAPE MEASURE
    public static double APRILTAG_Y_REL_FIELD_MM = 1013.5; //951.43 Tuned until zero | 1,013.5 mm IS FROM MEASURED FIELD WITH TAPE MEASURE


    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z = 300.232 / 2; // FROM CAD in camera system

    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X = 134.7705 - 25; // 369.541 FROM CAD in camera system

    public static final double GOAL_TO_APRIL_TAG_OFFSET_DISTANCE = Math.hypot(AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X, AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z);

    public static Position TURRET_REL_CAM_POS = new Position(-147.565, 0.977, 0);
    public static Point ROBOT_REL_TURRET_POINT = new Point(-24.51971493, 2.50233587);

    // BIOBUZZ tag IDs (manual 9.9), by Cell in each alliance's own frame: Cell A is the high-y Cell, B the low-y Cell.
    // The tags face down, on the bottom of the tipping Cells. Red: audience side 34-37 (A), far side 30-33 (B).
    // Blue: far side 42-45 (A), audience side 38-41 (B).
    public static int[] cellATagIds(AllianceColor allianceColor) {
        return allianceColor == AllianceColor.RED ? new int[]{34, 35, 36, 37} : new int[]{42, 43, 44, 45};
    }

    public static int[] cellBTagIds(AllianceColor allianceColor) {
        return allianceColor == AllianceColor.RED ? new int[]{30, 31, 32, 33} : new int[]{38, 39, 40, 41};
    }

    // Field pose of each tag above, in the alliance's own frame (same numbers for both alliances), index = position in the
    // ID arrays. null = not measured yet, and an unmeasured tag stays out of the layout.
    public static final Position[] CELL_A_TAG_POSES = new Position[4]; // TODO-measure
    public static final Position[] CELL_B_TAG_POSES = new Position[4]; // TODO-measure

    /** The tag the goal status looks for: the alliance's first Cell A tag. */
    public static int getGoalAprilTagId(AllianceColor allianceColor) {
        return cellATagIds(allianceColor)[0];
    }

    /**
     * The tags the robot may relocalize off: only the measured ones (a tag at a guessed pose relocalizes the robot to the
     * wrong place, see AprilTagFieldLayout). Built fresh on each call.
     */
    public static AprilTagFieldLayout buildFieldLayout(AllianceColor allianceColor) {
        return buildFieldLayout(cellATagIds(allianceColor), CELL_A_TAG_POSES, cellBTagIds(allianceColor), CELL_B_TAG_POSES);
    }

    /** Pure part of buildFieldLayout, so a test can pass poses of its own. */
    public static AprilTagFieldLayout buildFieldLayout(int[] aIds, Position[] aPoses, int[] bIds, Position[] bPoses) {
        AprilTagFieldLayout layout = new AprilTagFieldLayout();
        for (int i = 0; i < aIds.length; i++) if (aPoses[i] != null) layout.put(aIds[i], aPoses[i]);
        for (int i = 0; i < bIds.length; i++) if (bPoses[i] != null) layout.put(bIds[i], bPoses[i]);
        return layout;
    }
}
