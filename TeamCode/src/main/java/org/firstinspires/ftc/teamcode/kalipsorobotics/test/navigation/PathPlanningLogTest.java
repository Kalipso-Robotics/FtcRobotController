package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PathPlanning;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PickupPlan;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.List;

/**
 * No hardware. Run it, open FTC Dashboard in a browser, edit the values under PathPlanningLogTest
 * in the Configuration tab, and every change logs the planned waypoints to logcat (tag
 * "PathPlanningLog") and shows them on the Driver Hub. All values are mm in the FieldConfig frame.
 */
@Config
@TeleOp(name = "PathPlanningLogTest", group = "test")
public class PathPlanningLogTest extends LinearOpMode {
    private static final String TAG = "PathPlanningLog";

    public static double ROBOT_X = 500;
    public static double ROBOT_Y = 1800;
    public static double GOAL_X = 3100;
    public static double GOAL_Y = 1800;
    // TODO: tune. Half the robot's diagonal plus a margin.
    public static double BUFFER_MM = 350;

    // Ball pickup: the robot goes to ROBOT_X/ROBOT_Y and picks up the ball at BALL_X/BALL_Y.
    // AUTO true keeps the robot on its own side (manual G402).
    public static double BALL_X = 1100;
    public static double BALL_Y = 1800;
    public static boolean AUTO = false;
    // Half the robot's width, used as the midline buffer in AUTO.
    public static double ROBOT_HALF_MM = 229;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        telemetry.addData("how to", "edit values in FTC Dashboard > Configuration, then read logcat");
        telemetry.update();
        waitForStart();

        double[] last = null;
        while (opModeIsActive()) {
            double[] now = {ROBOT_X, ROBOT_Y, GOAL_X, GOAL_Y, BUFFER_MM, BALL_X, BALL_Y, AUTO ? 1 : 0, ROBOT_HALF_MM};
            if (!java.util.Arrays.equals(now, last)) {
                last = now;
                planAndLog(now);
            }
            idle();
        }
    }

    private void planAndLog(double[] in) {
        PathPlanning planner = PathPlanning.forField(in[4]);
        Point robot = new Point(in[0], in[1]);
        Point goal = new Point(in[2], in[3]);
        List<Point> path = planner.plan(robot, goal);

        String result = path == null ? "NO PATH" : path.toString();
        KLog.d(TAG, () -> "robot " + robot + " goal " + goal + " buffer " + in[4] + " -> " + result);
        telemetry.addData("robot", robot);
        telemetry.addData("goal", goal);
        telemetry.addData("path", result);

        PathPlanning pickupPlanner = in[7] == 1 ? PathPlanning.forFieldAuto(in[4], in[8]) : PathPlanning.forField(in[4]);
        Point ball = new Point(in[5], in[6]);
        PickupPlan pickup = pickupPlanner.planPickup(robot, ball);
        KLog.d(TAG, () -> "pickup ball " + ball + (in[7] == 1 ? " (AUTO)" : "") + " -> " + pickup
                + (pickup.isOk() ? " waypoints " + pickup.waypoints : ""));
        telemetry.addData("pickup", pickup.toString());
        telemetry.update();
    }
}
