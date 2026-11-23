package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

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

    private final static double SHOOT_POSITION_B_X = 1524;
    private final static double SHOOT_POSITION_B_Y = 609;
    public static Point bLaunchPoint =  new Point(SHOOT_POSITION_B_X, SHOOT_POSITION_B_Y);

    // The pickup target is this far from a Flower's center, straight out from its wall (the arm knocks balls off by
    // driving in, so likely smaller). Measured value, not Dashboard-tuned.
    public final static double FLOWER_BALL_OFFSET_MM = 150; // TODO-measure

    /** The closest of the four Flowers (A-D) to the robot. */
    public static Point nearestFlower(Point robot) {
        Point best = aFlowerPoint;
        for (Point f : new Point[]{bFlowerPoint, cFlowerPoint, dFlowerPoint}) {
            if (MathFunctions.distance(robot, f) < MathFunctions.distance(robot, best)) best = f;
        }
        return best;
    }

    /** Where to drive to pick up a Flower's balls: the Flower point moved FLOWER_BALL_OFFSET_MM from its nearest wall into the field. */
    public static Point flowerPilePoint(Point flower) {
        double x = flower.getX(), y = flower.getY();
        double toLowX = x, toHighX = FIELD_MM - x, toLowY = y, toHighY = FIELD_MM - y;
        double min = Math.min(Math.min(toLowX, toHighX), Math.min(toLowY, toHighY));
        if (min == toLowX) return new Point(x + FLOWER_BALL_OFFSET_MM, y);
        if (min == toHighX) return new Point(x - FLOWER_BALL_OFFSET_MM, y);
        if (min == toLowY) return new Point(x, y + FLOWER_BALL_OFFSET_MM);
        return new Point(x, y - FLOWER_BALL_OFFSET_MM);
    }

    // Pass: a low, flat shot UNDER the Hive (its lowest CELL edge is ~777 mm up, manual Figure 9-10), between the frame
    // legs, to the half of the field opposite the robot. The ball may roll on after landing; that is fine.
    /** Landing point distance from the far wall. Keeps the ball well inside the field. TODO-measure with PASS_RPS. */
    public final static double PASS_Y_FROM_WALL_MM = 1000;
    /** The landing point never gets closer than this to the side walls. */
    public final static double PASS_WALL_MARGIN_MM = 900;
    /** The pass line crosses the Hive middle at least this far inside each leg. */
    public final static double PASS_LEG_CLEARANCE_MM = 150;

    /**
     * Where a pass aims from the robot's position: on the half opposite the robot (no alliance flip: each alliance has
     * its own frame), on the line from the robot through the gap between the Hive legs, clamped inside the walls.
     * Straight across when the robot is already in line with the gap.
     */
    public static Point passTarget(Point robot) {
        double ty = robot.getY() >= HIVE_CENTER ? PASS_Y_FROM_WALL_MM : FIELD_MM - PASS_Y_FROM_WALL_MM;
        double crossX = MathFunctions.clamp(robot.getX(), HIVE_LEG_LOW_X[2] + PASS_LEG_CLEARANCE_MM, HIVE_LEG_HIGH_X[0] - PASS_LEG_CLEARANCE_MM);
        double toMiddle = HIVE_CENTER - robot.getY();
        double tx = Math.abs(toMiddle) < 1 ? crossX
                : robot.getX() + (crossX - robot.getX()) * (ty - robot.getY()) / toMiddle;
        return new Point(MathFunctions.clamp(tx, PASS_WALL_MARGIN_MM, FIELD_MM - PASS_WALL_MARGIN_MM), ty);
    }

    // Where the robot sits when it is zeroed against the loading-zone corner, facing +x (own frame). It is the
    // wheelbase centre, the point odometry reports, not the bumper corner. Also the start pose of every OpMode
    // that starts odometry fresh without an auto pose (OpModeUtilities.startOdometry(null)).
    // [2026-11-22] measured on the robot: 278 mm from the back wall, 188 mm from the side wall.
    public final static Position CORNER_ZERO_POSE = new Position(278, 188, 0);

    private final static double FIELD_MM = 3657.6;

    public final static double FLOWER_A_X = 100;  // nearly against the wall. The real center is 63.5 (half the 127 mm base), but the sim stack of 4 balls falls apart closer than about 80 mm
    private final static double FLOWER_A_Y = 2500;
    public static Point aFlowerPoint =  new Point(FLOWER_A_X, FLOWER_A_Y);

    public final static double FLOWER_B_X = 1219;
    private final static double FLOWER_B_Y = 100;   // same as FLOWER_A_X
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
