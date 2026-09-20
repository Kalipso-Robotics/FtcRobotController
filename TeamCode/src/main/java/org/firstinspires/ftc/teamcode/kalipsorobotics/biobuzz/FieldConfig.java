package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

public class FieldConfig {
    private final static double SHOOT_POSITION_A_X = 150;
    private final static double SHOOT_POSITION_A_Y = -50;
    public static Point aLaunchPoint =  new Point(SHOOT_POSITION_A_X, SHOOT_POSITION_A_Y);

    public final static double SHOOT_TARGET_A_X = 150;
    private final static double SHOOT_TARGET_A_Y = -50;
    public static Point aTargetPoint =  new Point(SHOOT_TARGET_A_X, SHOOT_TARGET_A_Y);

    private final static double SHOOT_POSITION_B_X = 150;
    private final static double SHOOT_POSITION_B_Y = -50;
    public static Point bLaunchPoint =  new Point(SHOOT_POSITION_B_X, SHOOT_POSITION_B_Y);

    public final static double SHOOT_TARGET_B_X = 150;
    private final static double SHOOT_TARGET_B_Y = -50;
    public static Point bTargetPoint =  new Point(SHOOT_TARGET_B_X, SHOOT_TARGET_B_Y);

    public final static double FLOWER_A_X = 150;
    private final static double FLOWER_A_Y = -50;
    public static Point aFlowerPoint =  new Point(FLOWER_A_X, FLOWER_A_Y);

    public final static double FLOWER_B_X = 150;
    private final static double FLOWER_B_Y = -50;
    public static Point bFlowerPoint =  new Point(FLOWER_B_X, FLOWER_B_Y);
}
