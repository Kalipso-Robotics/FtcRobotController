package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.List;
import java.util.Scanner;

/**
 * Laptop-only. Run this class's main, then type "robotX robotY goalX goalY" (mm, FieldConfig frame)
 * in the Run window and it prints the planned waypoints. Press Enter on an empty line to quit.
 * Optionally pass a buffer in mm as the first program argument (default 350).
 */
public class PathPlanningCli {
    public static void main(String[] args) {
        double buffer = args.length > 0 ? Double.parseDouble(args[0]) : 350;
        PathPlanning planner = PathPlanning.forField(buffer);
        System.out.println("Hive x " + FieldConfig.HIVE_MIN_X + ".." + FieldConfig.HIVE_MAX_X
                + ", y " + FieldConfig.HIVE_MIN_Y + ".." + FieldConfig.HIVE_MAX_Y + ", buffer " + buffer + " mm");

        Scanner in = new Scanner(System.in);
        while (true) {
            System.out.print("robotX robotY goalX goalY > ");
            if (!in.hasNextLine()) {
                return;
            }
            String line = in.nextLine().trim();
            if (line.isEmpty()) {
                return;
            }
            String[] parts = line.replace(",", " ").split("\\s+");
            if (parts.length != 4) {
                System.out.println("need 4 numbers");
                continue;
            }
            try {
                Point robot = new Point(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]));
                Point goal = new Point(Double.parseDouble(parts[2]), Double.parseDouble(parts[3]));
                List<Point> path = planner.plan(robot, goal);
                System.out.println(path == null ? "NO PATH (robot or goal inside the buffered Hive, or blocked)" : path.toString());
            } catch (NumberFormatException e) {
                System.out.println("not a number: " + e.getMessage());
            }
        }
    }
}
