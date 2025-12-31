package com.kalipsorobotics.actions.cameraVision;


import com.acmerobotics.dashboard.config.Config;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;

@Config
public class AprilTagConfig {

    public static double APRIL_TAG_HEADING_REL_FIELD_RAD = -Math.toRadians(126); //Math.toRadians(-234);

    //Tuned until zero field is different from CAD
    public static double APRILTAG_X_REL_FIELD_MM = 3076.6; // 3115 FROM TUNE | 3,076.6 FROM MEASURE FIELD WITH TAPE MEASURE
    public static double APRILTAG_Y_REL_FIELD_MM = 1013.5; //951.43 Tuned until zero | 1,013.5 mm IS FROM MEASURED FIELD WITH TAPE MEASURE


    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z = 300.232 / 2; // FROM CAD in camera system

    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X = 134.7705; // 369.541 FROM CAD in camera system
    public static double NEAR_DISTANCE_BEGIN_TO_SCALE_THRESHOLD = 1320;
    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAM_SPACE_NEAR_X = 84.7705;
    public static double APRIL_TAG_ANGLE_REL_TO_GOAL_RAD = Math.toRadians(54);

    public static final double GOAL_TO_APRIL_TAG_OFFSET_DISTANCE = Math.hypot(AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X, AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z);

    public static Position TURRET_REL_CAM_POS = new Position(-169.8848, -3.2466, 0);
    public static Point ROBOT_REL_TURRET_POINT = new Point(3.48052, 2.50233);


    public static double FIELD_ORIGIN_X_REL_APRILTAG_MM = 2629;
    public static double FIELD_ORIGIN_Y_REL_APRILTAG_MM = -1894;
}
