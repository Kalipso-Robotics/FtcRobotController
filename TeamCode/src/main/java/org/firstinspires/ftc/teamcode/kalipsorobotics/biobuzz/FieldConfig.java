package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

public class FieldConfig {
    // Each alliance has its own frame, with (0,0) at the corner of its own loading zone (field drawing
    // from the team): BLUE has (0,0) at the bottom-right corner of the manual's top view (audience at the
    // bottom), +x going left along the audience wall and +y going up. RED has (0,0) at the top-left
    // corner, +x going right and +y going down. The two frames are rotated 180 degrees from each other
    // and the field is symmetric under that rotation, so the numbers below (the garden at x~220,
    // y~3582, flower A, the Hive) are correct in BOTH frames and each alliance uses them as they are.

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

    // The other two flowers. The manual's top view (Figure 9-4) puts each flower 48 in from a corner,
    // going around the field like a pinwheel, so the field is symmetric under a 180 degree rotation,
    // not a mirror. C and D are A and B rotated 180 degrees about the field center.
    // VERIFY on the real field: the manual puts A at y = 96 in (2438.4 mm) from the audience wall.
    public static Point cFlowerPoint =  new Point(3657.6 - FLOWER_A_X, 3657.6 - FLOWER_A_Y);
    public static Point dFlowerPoint =  new Point(3657.6 - FLOWER_B_X, 3657.6 - FLOWER_B_Y);

    // Flower obstacle footprint for path planning: a square centered on each flower point.
    // The flower base is about 5 in by 4.5 in (measured by the team), so the square uses the longer
    // side. ASSUMPTION: the flower point is the center of the base. Measure from the wall to the base
    // center to confirm.
    public final static double FLOWER_SIZE_MM = 5 * 25.4;

    // Manual G402: during AUTO, columns A-C are the red side and D-F the blue side, split at the line
    // between columns C and D. In either alliance's own frame its own side is x < MIDLINE_X (its
    // loading zone is at x = 0), and the opposing side is x > MIDLINE_X.
    public final static double MIDLINE_X = 3657.6 / 2;

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

    // The frame is two legs at the X ends, joined by a crossbar well above the robot, so the robot can
    // drive between them along Y. Only the legs are floor-level obstacles; the hive cells hang about
    // 31 in above the floor (Onshape CAD views), so they are not obstacles for a robot of 18 in or
    // less.
    // Leg tube thickness at the floor: 2 in, measured by the team.
    public final static double HIVE_LEG_TUBE_MM = 2 * 25.4;
    // The legs also lean inward with height (about 9 in over the full 43.95 in, so about 4 in at 18 in
    // high; Onshape estimate). A leg is an obstacle everywhere the robot's body can reach it, so the leg
    // box grows inward by the lean at the robot's top height.
    // PLACEHOLDER: robot height not measured yet. 100 mm is for a robot about 18 in tall; a shorter
    // robot gives a smaller number.
    public final static double HIVE_LEG_LEAN_MM = 100;
    public final static double HIVE_LEG_THICKNESS_MM = HIVE_LEG_TUBE_MM + HIVE_LEG_LEAN_MM;

    /** Leg at the low-X end of the frame, {minX, minY, maxX, maxY}, un-inflated. */
    public final static double[] HIVE_LEG_LOW_X = {
            HIVE_MIN_X, HIVE_MIN_Y, HIVE_MIN_X + HIVE_LEG_THICKNESS_MM, HIVE_MAX_Y};

    /** Leg at the high-X end of the frame, {minX, minY, maxX, maxY}, un-inflated. */
    public final static double[] HIVE_LEG_HIGH_X = {
            HIVE_MAX_X - HIVE_LEG_THICKNESS_MM, HIVE_MIN_Y, HIVE_MAX_X, HIVE_MAX_Y};

    // Deadzone: a ball here cannot be picked up. A box in each of the four inside corners of the legs,
    // only in the area between the legs, because the diagonal beams and the rectangular robot get in
    // the way. PLACEHOLDER size, tune on the real frame.
    public final static double DEADZONE_MM = 108;

    /** The four deadzone boxes, each {minX, minY, maxX, maxY}. */
    public final static double[][] HIVE_DEADZONES = {
            {HIVE_LEG_LOW_X[2], HIVE_MIN_Y, HIVE_LEG_LOW_X[2] + DEADZONE_MM, HIVE_MIN_Y + DEADZONE_MM},
            {HIVE_LEG_LOW_X[2], HIVE_MAX_Y - DEADZONE_MM, HIVE_LEG_LOW_X[2] + DEADZONE_MM, HIVE_MAX_Y},
            {HIVE_LEG_HIGH_X[0] - DEADZONE_MM, HIVE_MIN_Y, HIVE_LEG_HIGH_X[0], HIVE_MIN_Y + DEADZONE_MM},
            {HIVE_LEG_HIGH_X[0] - DEADZONE_MM, HIVE_MAX_Y - DEADZONE_MM, HIVE_LEG_HIGH_X[0], HIVE_MAX_Y},
    };

    public final static double GARDEN_X = 220;
    private final static double GARDEN_Y = 3657.6-75;
    public static Point gardenPoint =  new Point(GARDEN_X, GARDEN_Y);
}
