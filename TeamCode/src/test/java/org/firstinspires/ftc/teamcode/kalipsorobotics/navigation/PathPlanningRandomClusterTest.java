package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

import java.util.List;
import java.util.Random;

/**
 * Throws random robot positions and random cluster locations at the planner using the real Hive
 * from FieldConfig, prints what it plans for each, and checks the plans are sane.
 * The seed is fixed so a failing case can be reproduced; change SEED to try a different set.
 */
public class PathPlanningRandomClusterTest {
    private static final long SEED = 42;
    private static final int TRIALS = 40;
    private static final double BUFFER_MM = 350;
    private static final double MARGIN_MM = 100;

    private final PathPlanning planner = PathPlanning.forField(BUFFER_MM);

    @Test
    public void randomClusters_printAndCheckPlans() {
        Random random = new Random(SEED);
        int direct = 0, oneCorner = 0, twoCorners = 0, noPath = 0;
        double worstDetourRatio = 1;

        System.out.println("buffered Hive box: x " + (FieldConfig.HIVE_MIN_X - BUFFER_MM) + ".."
                + (FieldConfig.HIVE_MAX_X + BUFFER_MM) + ", y " + (FieldConfig.HIVE_MIN_Y - BUFFER_MM)
                + ".." + (FieldConfig.HIVE_MAX_Y + BUFFER_MM));
        System.out.println("trial | robot -> cluster | route | path length / straight line | waypoints");

        for (int i = 0; i < TRIALS; i++) {
            Point robot = randomPoint(random);
            Point cluster = randomPoint(random);
            List<Point> path = planner.plan(robot, cluster);
            double straight = Point.distance(robot, cluster);

            if (path == null) {
                noPath++;
                System.out.printf("%3d | (%4.0f,%4.0f) -> (%4.0f,%4.0f) | NO PATH%n",
                        i, robot.getX(), robot.getY(), cluster.getX(), cluster.getY());
                continue;
            }

            double length = 0;
            Point previous = robot;
            for (Point next : path) {
                assertTrue("trial " + i + " blocked " + previous + " -> " + next, planner.clear(previous, next));
                length += Point.distance(previous, next);
                previous = next;
            }
            assertEquals("trial " + i + " must end at the cluster", cluster, path.get(path.size() - 1));
            assertTrue("trial " + i + " shorter than a straight line", length >= straight - 1e-6);

            double ratio = straight < 1 ? 1 : length / straight;
            worstDetourRatio = Math.max(worstDetourRatio, ratio);
            if (path.size() == 1) {
                direct++;
            } else if (path.size() == 2) {
                oneCorner++;
            } else {
                twoCorners++;
            }

            StringBuilder waypoints = new StringBuilder();
            for (Point p : path) {
                waypoints.append(String.format("(%.0f,%.0f) ", p.getX(), p.getY()));
            }
            System.out.printf("%3d | (%4.0f,%4.0f) -> (%4.0f,%4.0f) | %-10s | %.2fx | %s%n",
                    i, robot.getX(), robot.getY(), cluster.getX(), cluster.getY(),
                    path.size() == 1 ? "direct" : (path.size() - 1) + " corner(s)", ratio, waypoints);
        }

        System.out.printf("summary: direct=%d, 1 corner=%d, 2 corners=%d, no path=%d, worst detour=%.2fx%n",
                direct, oneCorner, twoCorners, noPath, worstDetourRatio);
    }

    private static Point randomPoint(Random random) {
        double range = PathPlanning.FIELD_SIZE_MM - 2 * MARGIN_MM;
        return new Point(MARGIN_MM + random.nextDouble() * range, MARGIN_MM + random.nextDouble() * range);
    }
}
