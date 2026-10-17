package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Plans detour waypoints around axis-aligned boxes (the Hive and the flowers).
 * Every box is grown by the buffer. Each buffered box contributes four detour corners, pushed out
 * by CORNER_OFFSET_MM. plan() returns the shortest path from the robot to the goal that never
 * enters a buffered box, moving in straight lines between the robot, detour corners and the goal.
 * All units are mm in the FieldConfig frame (garden corner is (0,0)).
 *
 * The corners and the links between them only depend on the boxes and the buffer, so they are
 * built once in the constructor. Only the links to the robot and the goal happen in plan().
 */
public class PathPlanning {
    public static final double FIELD_SIZE_MM = 3657.6;

    /** How far past the buffered corner the detour point is pushed, on each axis. */
    public static double CORNER_OFFSET_MM = 20;

    /** Lets a segment run exactly along the buffer edge without counting as a hit. */
    private static final double EDGE_TOLERANCE_MM = 1;

    /** Buffered boxes, each {minX, minY, maxX, maxY}. */
    private final List<double[]> boxes = new ArrayList<>();

    /** The real obstacles (not grown), used to check the robot body when picking up a ball. */
    private final List<double[]> hardBoxes = new ArrayList<>();

    /** A ball inside one of these cannot be picked up. Also checked against the robot body. */
    private final List<double[]> deadzones = new ArrayList<>();

    /** AUTO: the robot body must stay on its own side of the midline (manual G402). */
    private final boolean autoSide;

    /** Directions the robot may face when it picks up a ball, evenly spaced around the circle. */
    private static final int APPROACH_DIRECTIONS = 24;

    /** Extra room around real obstacles when the robot body is checked, for tracking error. */
    public static double PICKUP_MARGIN_MM = 20;

    private static final double RING_STEP_MM = 10;
    private static final double MAX_RING_STRETCH_MM = 600;

    /** How far the robot may drive to get out of a buffer ring it starts inside. */
    private static final double MAX_ESCAPE_MM = 800;

    /** Detour corners that are on the field and not inside any buffered box. */
    private final List<Point> corners = new ArrayList<>();

    /** cornerDist[i][j] is the distance between corners i and j, or -1 if the segment is blocked. */
    private final double[][] cornerDist;

    /** One box, the Hive for example. */
    public PathPlanning(double minX, double minY, double maxX, double maxY, double bufferMM) {
        this(bufferMM, new double[]{minX, minY, maxX, maxY});
    }

    /**
     * The two Hive legs (the robot can drive between them) and both flowers from FieldConfig.
     * Crossing to the opposing alliance's side is allowed (use this one in TELEOP).
     */
    public static PathPlanning forField(double bufferMM) {
        return new PathPlanning(fieldBoxes(bufferMM), Arrays.asList(fieldRaw()),
                Arrays.asList(FieldConfig.HIVE_DEADZONES), false);
    }

    /**
     * Same as forField, plus the whole opposing half of the field as an obstacle (manual G402, AUTO).
     * The robot is planned in its own alliance's frame, so the blocked half is x > FieldConfig.MIDLINE_X.
     * @param sideBufferMM how far the robot center must stay from the midline. Use the robot's
     *                     half-width to keep the whole body on its own side.
     */
    public static PathPlanning forFieldAuto(double bufferMM, double sideBufferMM) {
        List<double[]> boxes = fieldBoxes(bufferMM);
        boxes.add(grown(new double[]{FieldConfig.MIDLINE_X, 0, FIELD_SIZE_MM, FIELD_SIZE_MM}, sideBufferMM));
        return new PathPlanning(boxes, Arrays.asList(fieldRaw()), Arrays.asList(FieldConfig.HIVE_DEADZONES), true);
    }

    /** Real Hive legs and flowers, not grown. */
    private static double[][] fieldRaw() {
        double half = FieldConfig.FLOWER_SIZE_MM / 2;
        Point[] flowers = {FieldConfig.aFlowerPoint, FieldConfig.bFlowerPoint,
                FieldConfig.cFlowerPoint, FieldConfig.dFlowerPoint};
        double[][] result = new double[2 + flowers.length][];
        result[0] = FieldConfig.HIVE_LEG_LOW_X;
        result[1] = FieldConfig.HIVE_LEG_HIGH_X;
        for (int i = 0; i < flowers.length; i++) {
            Point f = flowers[i];
            result[2 + i] = new double[]{f.getX() - half, f.getY() - half, f.getX() + half, f.getY() + half};
        }
        return result;
    }

    /** Buffered Hive legs and flowers. */
    private static List<double[]> fieldBoxes(double bufferMM) {
        return buffered(bufferMM, fieldRaw());
    }

    private static double[] grown(double[] box, double bufferMM) {
        return new double[]{box[0] - bufferMM, box[1] - bufferMM, box[2] + bufferMM, box[3] + bufferMM};
    }

    private static List<double[]> buffered(double bufferMM, double[]... rawBoxes) {
        List<double[]> result = new ArrayList<>();
        for (double[] box : rawBoxes) {
            result.add(grown(box, bufferMM));
        }
        return result;
    }

    /** @param rawBoxes un-buffered boxes, each {minX, minY, maxX, maxY} */
    public PathPlanning(double bufferMM, double[]... rawBoxes) {
        this(buffered(bufferMM, rawBoxes), Arrays.asList(rawBoxes), new ArrayList<double[]>(), false);
    }

    /**
     * @param bufferedBoxes boxes already grown by their buffer, each {minX, minY, maxX, maxY}
     * @param hardBoxes     the real obstacles, not grown
     * @param deadzones     balls inside these cannot be picked up
     * @param autoSide      true to keep the robot body on its own side of the midline
     */
    private PathPlanning(List<double[]> bufferedBoxes, List<double[]> hardBoxes, List<double[]> deadzones,
                         boolean autoSide) {
        boxes.addAll(bufferedBoxes);
        this.hardBoxes.addAll(hardBoxes);
        this.deadzones.addAll(deadzones);
        this.autoSide = autoSide;

        for (double[] box : boxes) {
            addCorner(box[0] - CORNER_OFFSET_MM, box[1] - CORNER_OFFSET_MM);
            addCorner(box[2] + CORNER_OFFSET_MM, box[1] - CORNER_OFFSET_MM);
            addCorner(box[2] + CORNER_OFFSET_MM, box[3] + CORNER_OFFSET_MM);
            addCorner(box[0] - CORNER_OFFSET_MM, box[3] + CORNER_OFFSET_MM);
        }

        int n = corners.size();
        cornerDist = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                boolean linked = i != j && clear(corners.get(i), corners.get(j));
                cornerDist[i][j] = linked ? Point.distance(corners.get(i), corners.get(j)) : -1;
            }
        }
    }

    private void addCorner(double x, double y) {
        if (x < 0 || x > FIELD_SIZE_MM || y < 0 || y > FIELD_SIZE_MM) {
            return;
        }
        Point corner = new Point(x, y);
        if (!insideBuffer(corner)) {
            corners.add(corner);
        }
    }

    /**
     * Plans to the cluster published in SharedData.
     * @return null if no target cluster is set, otherwise the same as plan()
     */
    public List<Point> planToTargetCluster(Point robot) {
        Point target = SharedData.getTargetClusterXY();
        if (target == null) {
            return null;
        }
        return plan(robot, target);
    }

    /**
     * Plans how to pick up a ball. The intake grabs the ball in front of the robot center, so the
     * center only has to get to a stand-off point GRAB_MM short of the ball (or pullInMm() short, with
     * the front touching the ball, when the ball is too close to an obstacle for that). For each of
     * APPROACH_DIRECTIONS facing directions, the stand-off is checked: the rectangular robot body must
     * not touch a real obstacle (or a deadzone, or in AUTO the opposing side). First choice is a
     * stand-off outside every buffer ring, planned with plan(). If none works, the stand-off may be
     * inside a ring: the path goes to the ring edge with plan() and then drives straight in, with the
     * body checked every RING_STEP_MM. The shortest path wins.
     */
    public PickupPlan planPickup(Point robot, Point ball) {
        double x = ball.getX();
        double y = ball.getY();
        if (x < 0 || x > FIELD_SIZE_MM || y < 0 || y > FIELD_SIZE_MM) {
            return PickupPlan.fail(PickupPlan.Reason.OFF_FIELD, "ball " + ball + " is off the field");
        }
        if (autoSide && x > FieldConfig.MIDLINE_X) {
            return PickupPlan.fail(PickupPlan.Reason.OPPOSING_SIDE, "ball " + ball + " is on the opposing side");
        }
        if (inside(ball, deadzones)) {
            return PickupPlan.fail(PickupPlan.Reason.IN_DEADZONE, "ball " + ball + " is in a deadzone");
        }
        if (inside(ball, hardBoxes)) {
            return PickupPlan.fail(PickupPlan.Reason.INSIDE_OBSTACLE, "ball " + ball + " is inside an obstacle");
        }

        boolean anyApproach = false;
        PickupPlan best = null;
        double bestLength = Double.MAX_VALUE;
        // Prefer the ball ending up inside the intake mouth (mode 0). Only if that finds nothing, stop
        // with the front touching the ball and let the intake pull it in (mode 1, the ball may be
        // against a leg). Within a mode, prefer a stand-off outside every ring (pass 0) over one
        // inside a ring (pass 1).
        for (int mode = 0; mode < 2 && best == null; mode++) {
            boolean pullIn = mode == 1;
            double distance = pullIn ? RobotFootprint.pullInMm() : RobotFootprint.GRAB_MM;
            for (int pass = 0; pass < 2 && best == null; pass++) {
                for (int i = 0; i < APPROACH_DIRECTIONS; i++) {
                    double heading = 2 * Math.PI * i / APPROACH_DIRECTIONS;
                    double cos = Math.cos(heading);
                    double sin = Math.sin(heading);
                    Point standOff = new Point(x - distance * cos, y - distance * sin);
                    if (bodyBlocked(standOff, heading)) {
                        continue;
                    }
                    anyApproach = true;

                    boolean inRing = insideBuffer(standOff);
                    if (inRing != (pass == 1)) {
                        continue;
                    }
                    List<Point> path;
                    if (!inRing) {
                        path = plan(robot, standOff);
                        if (path == null) {
                            continue;
                        }
                    } else {
                        Point entry = ringEntry(standOff, heading);
                        if (entry == null) {
                            continue;
                        }
                        List<Point> toEntry = plan(robot, entry);
                        if (toEntry == null) {
                            continue;
                        }
                        path = new ArrayList<>(toEntry);
                        path.add(standOff);
                    }

                    double length = pathLength(robot, path);
                    if (length < bestLength) {
                        bestLength = length;
                        String why = String.format(Locale.US, "stand-off %s facing %.0f deg, %s, %s, path %.0f mm",
                                standOff, Math.toDegrees(heading), pullIn ? "front touches the ball (pull in)" : "ball in the mouth",
                                inRing ? "drives into the ring" : "outside the ring", length);
                        best = PickupPlan.ok(path, standOff, heading, inRing, pullIn, why);
                    }
                }
            }
        }

        if (best != null) {
            return best;
        }
        if (anyApproach) {
            return PickupPlan.fail(PickupPlan.Reason.NO_PATH, "no route to any stand-off for ball " + ball);
        }
        return PickupPlan.fail(PickupPlan.Reason.NO_APPROACH, "the robot body hits an obstacle at every angle for ball " + ball);
    }

    /**
     * Walks back from the stand-off, against the facing direction, to the first point outside every
     * buffer ring. Null if the body hits something on the way or the walk leaves the field.
     */
    private Point ringEntry(Point standOff, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        for (double t = 0; t <= MAX_RING_STRETCH_MM; t += RING_STEP_MM) {
            Point p = new Point(standOff.getX() - t * cos, standOff.getY() - t * sin);
            if (bodyBlocked(p, heading)) {
                return null;
            }
            if (!insideBuffer(p)) {
                return p;
            }
        }
        return null;
    }

    /** True if the robot body, centered at c and facing heading, hits something it must not. */
    private boolean bodyBlocked(Point c, double heading) {
        for (double[] corner : RobotFootprint.corners(c, heading)) {
            if (corner[0] < 0 || corner[0] > FIELD_SIZE_MM || corner[1] < 0 || corner[1] > FIELD_SIZE_MM) {
                return true;
            }
            if (autoSide && corner[0] > FieldConfig.MIDLINE_X + EDGE_TOLERANCE_MM) {
                return true;
            }
        }
        for (double[] box : hardBoxes) {
            if (RobotFootprint.overlaps(c, heading, box, PICKUP_MARGIN_MM)) {
                return true;
            }
        }
        for (double[] box : deadzones) {
            if (RobotFootprint.overlaps(c, heading, box, PICKUP_MARGIN_MM)) {
                return true;
            }
        }
        return false;
    }

    private static double pathLength(Point start, List<Point> path) {
        double length = 0;
        Point previous = start;
        for (Point next : path) {
            length += Point.distance(previous, next);
            previous = next;
        }
        return length;
    }

    /**
     * @return waypoints after the robot, ending at the goal (a single point means drive direct),
     * or null if there is no path (goal inside a buffered box, robot inside a real obstacle, or every
     * route blocked). A robot that starts inside a buffer ring first gets a waypoint straight out to
     * open floor, then the route from there.
     */
    public List<Point> plan(Point robot, Point goal) {
        if (insideBuffer(goal)) {
            return null;
        }
        if (insideBuffer(robot)) {
            Point out = escapePoint(robot);
            if (out == null) {
                return null;
            }
            List<Point> rest = plan(out, goal);
            if (rest == null) {
                return null;
            }
            List<Point> path = new ArrayList<>();
            path.add(out);
            path.addAll(rest);
            return path;
        }
        if (clear(robot, goal)) {
            return Collections.singletonList(goal);
        }

        // Dijkstra over the corners. Index n is the goal; the robot is the start.
        int n = corners.size();
        double[] best = new double[n + 1];
        int[] previous = new int[n + 1];
        boolean[] done = new boolean[n + 1];
        Arrays.fill(best, Double.MAX_VALUE);
        Arrays.fill(previous, -1);

        for (int i = 0; i < n; i++) {
            if (clear(robot, corners.get(i))) {
                best[i] = Point.distance(robot, corners.get(i));
            }
        }

        while (true) {
            int current = -1;
            for (int i = 0; i <= n; i++) {
                if (!done[i] && best[i] < Double.MAX_VALUE && (current == -1 || best[i] < best[current])) {
                    current = i;
                }
            }
            if (current == -1) {
                return null;
            }
            if (current == n) {
                break;
            }
            done[current] = true;

            for (int next = 0; next < n; next++) {
                double d = cornerDist[current][next];
                if (d >= 0 && !done[next] && best[current] + d < best[next]) {
                    best[next] = best[current] + d;
                    previous[next] = current;
                }
            }
            if (clear(corners.get(current), goal)) {
                double length = best[current] + Point.distance(corners.get(current), goal);
                if (length < best[n]) {
                    best[n] = length;
                    previous[n] = current;
                }
            }
        }

        List<Point> path = new ArrayList<>();
        path.add(goal);
        for (int i = previous[n]; i != -1; i = previous[i]) {
            path.add(corners.get(i));
        }
        Collections.reverse(path);

        // Drop detour points the straight line can skip, for example a corner in the middle of a
        // straight run. This never makes the path longer.
        Point from = robot;
        int i = 0;
        while (i < path.size() - 1) {
            if (clear(from, path.get(i + 1))) {
                path.remove(i);
            } else {
                from = path.get(i);
                i++;
            }
        }
        return path;
    }

    /**
     * The nearest open-floor point for a robot that is inside a buffer ring: outside every buffer, on
     * the field, and reachable in a straight line without touching a real obstacle. Null if the robot is
     * inside a real obstacle or nothing within MAX_ESCAPE_MM works.
     */
    private Point escapePoint(Point robot) {
        if (inside(robot, hardBoxes)) {
            return null;
        }
        for (double t = RING_STEP_MM; t <= MAX_ESCAPE_MM; t += RING_STEP_MM) {
            for (int i = 0; i < APPROACH_DIRECTIONS; i++) {
                double angle = 2 * Math.PI * i / APPROACH_DIRECTIONS;
                Point p = new Point(robot.getX() + t * Math.cos(angle), robot.getY() + t * Math.sin(angle));
                if (p.getX() < 0 || p.getX() > FIELD_SIZE_MM || p.getY() < 0 || p.getY() > FIELD_SIZE_MM
                        || insideBuffer(p)) {
                    continue;
                }
                boolean clearOfObstacles = true;
                for (double[] box : hardBoxes) {
                    if (!clear(robot, p, box)) {
                        clearOfObstacles = false;
                        break;
                    }
                }
                if (clearOfObstacles) {
                    return p;
                }
            }
        }
        return null;
    }

    /** True if the point is strictly inside any buffered box. */
    boolean insideBuffer(Point p) {
        return inside(p, boxes);
    }

    /** True if the point is strictly inside any of the boxes. */
    private static boolean inside(Point p, List<double[]> list) {
        for (double[] box : list) {
            if (p.getX() > box[0] + EDGE_TOLERANCE_MM && p.getX() < box[2] - EDGE_TOLERANCE_MM
                    && p.getY() > box[1] + EDGE_TOLERANCE_MM && p.getY() < box[3] - EDGE_TOLERANCE_MM) {
                return true;
            }
        }
        return false;
    }

    /** True if the segment a to b does not enter any buffered box. */
    boolean clear(Point a, Point b) {
        for (double[] box : boxes) {
            if (!clear(a, b, box)) {
                return false;
            }
        }
        return true;
    }

    /** Slab test against one buffered box. */
    private static boolean clear(Point a, Point b, double[] box) {
        double minX = box[0] + EDGE_TOLERANCE_MM;
        double minY = box[1] + EDGE_TOLERANCE_MM;
        double maxX = box[2] - EDGE_TOLERANCE_MM;
        double maxY = box[3] - EDGE_TOLERANCE_MM;

        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        double tEnter = 0;
        double tExit = 1;

        // x slab
        if (dx == 0) {
            if (a.getX() <= minX || a.getX() >= maxX) {
                return true;
            }
        } else {
            double t1 = (minX - a.getX()) / dx;
            double t2 = (maxX - a.getX()) / dx;
            tEnter = Math.max(tEnter, Math.min(t1, t2));
            tExit = Math.min(tExit, Math.max(t1, t2));
        }

        // y slab
        if (dy == 0) {
            if (a.getY() <= minY || a.getY() >= maxY) {
                return true;
            }
        } else {
            double t1 = (minY - a.getY()) / dy;
            double t2 = (maxY - a.getY()) / dy;
            tEnter = Math.max(tEnter, Math.min(t1, t2));
            tExit = Math.min(tExit, Math.max(t1, t2));
        }

        // overlap in t means the segment passes through the box interior
        return tEnter >= tExit;
    }
}
