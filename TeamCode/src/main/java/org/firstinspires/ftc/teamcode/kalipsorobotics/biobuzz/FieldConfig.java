package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

public class FieldConfig {
    // Garden corner is (0,0)

    private final static double SHOOT_POSITION_A_X = 1524;
    private final static double SHOOT_POSITION_A_Y = 3657.6-609;
    public static Point aLaunchPoint =  new Point(SHOOT_POSITION_A_X, SHOOT_POSITION_A_Y);

    public final static double SHOOT_TARGET_A_X = 1524;
    private final static double SHOOT_TARGET_A_Y = 3657.6-1524;
    public static Point aTargetPoint =  new Point(SHOOT_TARGET_A_X, SHOOT_TARGET_A_Y);

    private final static double SHOOT_POSITION_B_X = 1524;
    private final static double SHOOT_POSITION_B_Y = 609;
    public static Point bLaunchPoint =  new Point(SHOOT_POSITION_B_X, SHOOT_POSITION_B_Y);

    public final static double SHOOT_TARGET_B_X = 1524;
    private final static double SHOOT_TARGET_B_Y = 3657.6-2135;
    public static Point bTargetPoint =  new Point(SHOOT_TARGET_B_X, SHOOT_TARGET_B_Y);

    public final static double FLOWER_A_X = 225;
    private final static double FLOWER_A_Y = 2500;
    public static Point aFlowerPoint =  new Point(FLOWER_A_X, FLOWER_A_Y);

    public final static double FLOWER_B_X = 1219;
    private final static double FLOWER_B_Y = 3657.6-3500;
    public static Point bFlowerPoint =  new Point(FLOWER_B_X, FLOWER_B_Y);

    // Flower obstacle footprint for path planning: a square centered on each flower point.
    // PLACEHOLDER: measure the real flower (plus anything that sticks out) and update.
    public final static double FLOWER_SIZE_MM = 200;

    // Hive frame footprint (axis-aligned, un-inflated), centered on the field.
    // Game manual 9.6.1: frame is 49.46 in wide (along X) by 38.95 in deep (along Y) at its base.
    // Orientation from Figure 9-4. NOMINAL: confirm with a tape measure on the real field, and note
    // the tipped CELLS can stick out past this footprint, so keep BUFFER_MM generous.
    private final static double HIVE_CENTER = 3657.6 / 2;
    private final static double HIVE_HALF_WIDTH_X = 49.46 * 25.4 / 2;
    private final static double HIVE_HALF_DEPTH_Y = 38.95 * 25.4 / 2;
    public final static double HIVE_MIN_X = HIVE_CENTER - HIVE_HALF_WIDTH_X;
    public final static double HIVE_MIN_Y = HIVE_CENTER - HIVE_HALF_DEPTH_Y;
    public final static double HIVE_MAX_X = HIVE_CENTER + HIVE_HALF_WIDTH_X;
    public final static double HIVE_MAX_Y = HIVE_CENTER + HIVE_HALF_DEPTH_Y;

    public final static double GARDEN_X = 220;
    private final static double GARDEN_Y = 3657.6-75;
    public static Point gardenPoint =  new Point(GARDEN_X, GARDEN_Y);
}
