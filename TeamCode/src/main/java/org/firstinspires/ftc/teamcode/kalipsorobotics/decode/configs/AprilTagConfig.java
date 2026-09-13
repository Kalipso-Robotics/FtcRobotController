package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagFieldLayout;

@Config
public class AprilTagConfig {


    public static double APRIL_TAG_HEADING_REL_FIELD_RAD = -Math.toRadians(125.54); //Math.toRadians(-234);

    //Tuned until zero field is different from CAD
    public static double APRILTAG_X_REL_FIELD_MM = 3076.6; // 3115 FROM TUNE | 3,076.6 FROM MEASURE FIELD WITH TAPE MEASURE
    public static double APRILTAG_Y_REL_FIELD_MM = 1013.5; //951.43 Tuned until zero | 1,013.5 mm IS FROM MEASURED FIELD WITH TAPE MEASURE

    public static int RED_GOAL_APRILTAG_ID = 24;
    public static int BLUE_GOAL_APRILTAG_ID = 20;

    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z = 300.232 / 2; // FROM CAD in camera system

    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X = 134.7705 - 25; // 369.541 FROM CAD in camera system

    public static final double GOAL_TO_APRIL_TAG_OFFSET_DISTANCE = Math.hypot(AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X, AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z);

    public static Position TURRET_REL_CAM_POS = new Position(-147.565, 0.977, 0);
    public static Point ROBOT_REL_TURRET_POINT = new Point(-24.51971493, 2.50233587);

    public static int getGoalAprilTagId(AllianceColor allianceColor) {
        return allianceColor == AllianceColor.RED ? RED_GOAL_APRILTAG_ID : BLUE_GOAL_APRILTAG_ID;
    }

    /**
     * Builds the field's known tag layout from the current (dashboard-tunable) constants
     * above. Called fresh rather than cached statically so live tuning still takes effect.
     *
     * THIS LIST IS THE ROBOT'S PER-TAG TRANSFORM TABLE.
     *   AprilTagDetectionAction relocalizes off ANY tag listed here, choosing the nearest
     *   one it can see. Both goal tags are listed, so a red robot that can only see the
     *   blue goal tag still knows where it is.
     *
     * ADDING A TAG (e.g. the DECODE obelisk, or a tag taped to a practice-field wall):
     *   Measure the tag's field pose, then add one line:
     *
     *       .put(22, new Position(xMM, yMM, headingRad))
     *
     *   x/y are the tag centre in field coordinates (+X forward from init, +Y right) and
     *   heading is the direction its printed face looks along, CCW radians. Nothing else
     *   changes - the action picks it up automatically.
     *
     *   Leave a tag OUT until it is actually measured. An unlisted tag is detected and
     *   ignored, which is safe; a tag listed at a guessed pose confidently relocalizes the
     *   robot to the wrong place, which is not.
     */
    public static AprilTagFieldLayout buildFieldLayout() {
        return new AprilTagFieldLayout()
                .put(RED_GOAL_APRILTAG_ID, new Position(APRILTAG_X_REL_FIELD_MM, APRILTAG_Y_REL_FIELD_MM, APRIL_TAG_HEADING_REL_FIELD_RAD))
                .put(BLUE_GOAL_APRILTAG_ID, new Position(APRILTAG_X_REL_FIELD_MM, -APRILTAG_Y_REL_FIELD_MM, -APRIL_TAG_HEADING_REL_FIELD_RAD));
    }

}
