package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PathPlanning;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.List;

/**
 * Drives the PathPlanning detour around the Hive, from a start pose on one side of it to a goal on
 * the other. Waypoints are shown in telemetry during init so they can be checked against taped
 * Hive lines before pressing start. Tape the real (un-buffered) Hive outline on the floor first.
 * Telemetry reports the closest the robot center got to the Hive box.
 */
@Autonomous(name = "TestPathPlanning", group = "test")
public class TestPathPlanning extends KOpMode {
    // TODO: tune. Half the robot's diagonal plus a margin.
    private static final double BUFFER_MM = 350;

    // Start left of the Hive and goal right of it (opposite sides -> 2 corners). Edit to test the
    // direct and 1-corner cases.
    private static final double HIVE_CENTER_Y = (FieldConfig.HIVE_MIN_Y + FieldConfig.HIVE_MAX_Y) / 2;
    private static final double START_X = FieldConfig.HIVE_MIN_X - 700;
    private static final double START_Y = HIVE_CENTER_Y;
    private static final double GOAL_X = FieldConfig.HIVE_MAX_X + 700;
    private static final double GOAL_Y = HIVE_CENTER_Y;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        telemetry.setAutoClear(false);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule,
                START_X, START_Y, Math.toRadians(0));
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        PathPlanning planner = PathPlanning.forField(BUFFER_MM);
        // The planner goes to the target cluster in SharedData. Normally the ball detection sets it;
        // fall back to the hardcoded goal so this test runs on its own.
        if (SharedData.getTargetClusterXY() == null) {
            SharedData.setTargetClusterXY(new Point(GOAL_X, GOAL_Y));
        }
        Point goal = SharedData.getTargetClusterXY();
        List<Point> waypoints = planner.planToTargetCluster(new Point(START_X, START_Y));

        AdaptivePurePursuitAction follow = new AdaptivePurePursuitAction(driveTrain);
        if (waypoints == null) {
            telemetry.addData("plan", "NO PATH, will not drive");
            KLog.d("TestPathPlanning", "NO PATH");
        } else {
            telemetry.addData("plan", waypoints.size() + " waypoint(s)");
            for (int i = 0; i < waypoints.size(); i++) {
                Point p = waypoints.get(i);
                telemetry.addData("wp" + i, "(%.0f, %.0f)", p.getX(), p.getY());
                KLog.d("TestPathPlanning", () -> "waypoint " + p);
                follow.addPoint(p.getX(), p.getY(), 0);
            }
        }
        telemetry.addData("start", "(%.0f, %.0f)", START_X, START_Y);
        telemetry.addData("goal", "(%.0f, %.0f)", goal.getX(), goal.getY());
        telemetry.update();

        waitForStartPrecomputingPaths();

        double minClearance = Double.MAX_VALUE;
        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            if (waypoints != null && !follow.getIsDone()) {
                follow.updateCheckDone();
            }

            Position pose = SharedData.getOdometryWheelIMUPosition();
            double clearance = distanceToHive(pose.getX(), pose.getY());
            minClearance = Math.min(minClearance, clearance);
            KLog.d("TestPathPlanning", () -> "pose " + pose + " hiveDist " + clearance);
            telemetry.addData("pose", "(%.0f, %.0f)", pose.getX(), pose.getY());
            telemetry.addData("min robot-center to Hive (mm)", "%.0f", minClearance);
            telemetry.update();
        }
        cleanupRobot();
    }

    /** Distance from a point to the raw (un-buffered) Hive box; 0 if inside. */
    private static double distanceToHive(double x, double y) {
        double dx = Math.max(Math.max(FieldConfig.HIVE_MIN_X - x, 0), x - FieldConfig.HIVE_MAX_X);
        double dy = Math.max(Math.max(FieldConfig.HIVE_MIN_Y - y, 0), y - FieldConfig.HIVE_MAX_Y);
        return Math.hypot(dx, dy);
    }
}
