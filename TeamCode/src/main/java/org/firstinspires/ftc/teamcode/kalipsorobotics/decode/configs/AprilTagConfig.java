package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

@Config
public class AprilTagConfig {


    public static double APRIL_TAG_HEADING_REL_FIELD_RAD = -Math.toRadians(125.54); //Math.toRadians(-234);

    //Tuned until zero field is different from CAD
    public static double APRILTAG_X_REL_FIELD_MM = 3076.6; // 3115 FROM TUNE | 3,076.6 FROM MEASURE FIELD WITH TAPE MEASURE
    public static double APRILTAG_Y_REL_FIELD_MM = 1013.5 - 76.2; //951.43 Tuned until zero | 1,013.5 mm IS FROM MEASURED FIELD WITH TAPE MEASURE


    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z = 300.232 / 2; // FROM CAD in camera system

    public static double GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X = 134.7705 - 25; // 369.541 FROM CAD in camera system

    public static final double GOAL_TO_APRIL_TAG_OFFSET_DISTANCE = Math.hypot(AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_X, AprilTagConfig.GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z);

    public static Position TURRET_REL_CAM_POS = new Position(-147.565, 0.977, 0);
    public static Point ROBOT_REL_TURRET_POINT = new Point(-24.51971493, 2.50233587);

}
