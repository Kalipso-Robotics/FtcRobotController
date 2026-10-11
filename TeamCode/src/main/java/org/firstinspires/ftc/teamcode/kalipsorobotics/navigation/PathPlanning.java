package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

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
        return new PathPlanning(fieldBoxes(bufferMM));
    }

    /**
     * Same as forField, plus the whole opposing half of the field as an obstacle (manual G402, AUTO).
     * The robot is planned in the blue frame, so the blocked half is x > FieldConfig.MIDLINE_X.
     * @param sideBufferMM how far the robot center must stay from the midline. Use the robot's
     *                     half-width to keep the whole body on its own side.
     */
    public static PathPlanning forFieldAuto(double bufferMM, double sideBufferMM) {
        List<double[]> boxes = fieldBoxes(bufferMM);
        boxes.add(grown(new double[]{FieldConfig.MIDLINE_X, 0, FIELD_SIZE_MM, FIELD_SIZE_MM}, sideBufferMM));
        return new PathPlanning(boxes);
    }

    /** Buffered Hive legs and flowers. */
    private static List<double[]> fieldBoxes(double bufferMM) {
        double half = FieldConfig.FLOWER_SIZE_MM / 2;
        Point a = FieldConfig.aFlowerPoint;
        Point b = FieldConfig.bFlowerPoint;
        return buffered(bufferMM,
                FieldConfig.HIVE_LEG_LOW_X,
                FieldConfig.HIVE_LEG_HIGH_X,
                new double[]{a.getX() - half, a.getY() - half, a.getX() + half, a.getY() + half},
                new double[]{b.getX() - half, b.getY() - half, b.getX() + half, b.getY() + half});
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
        this(buffered(bufferMM, rawBoxes));
    }

    /** @param bufferedBoxes boxes that are already grown by their buffer, each {minX, minY, maxX, maxY} */
    private PathPlanning(List<double[]> bufferedBoxes) {
        boxes.addAll(bufferedBoxes);

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
     * @return waypoints after the robot, ending at the goal (a single point means drive direct),
     * or null if there is no path (robot or goal inside a buffered box, or every route blocked).
     */
    public List<Point> plan(Point robot, Point goal) {
        if (insideBuffer(robot) || insideBuffer(goal)) {
            return null;
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
        return path;
    }

    /** True if the point is strictly inside any buffered box. */
    private boolean insideBuffer(Point p) {
        for (double[] box : boxes) {
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
