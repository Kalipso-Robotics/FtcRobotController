package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

import java.util.ArrayList;
import java.util.List;

/**
 * Picks the best group of balls to drive to next.
 *
 * How:
 *   1. Drop balls on the other alliance's half (AUTO rule G402: the planner would refuse them anyway).
 *   2. Chain balls into groups: a ball closer than GROUP_MM to any ball of a group joins it
 *      (so balls touching across a tile line are one group, and a line of balls is one group).
 *   3. Drop groups holding an opponent or UNKNOWN ball: driving through them would swallow it.
 *   4. Score each group by points per second: our nectar 2, pollen 1, capped by the room left in the
 *      intake, divided by drive time plus a stop. A group that does not fill the intake may add a second
 *      group after it (the score of the best pair counts for the first group), so a small group next to a
 *      big one beats a lone ball far away.
 *   5. Return the best group; drive to aimPointOf(it).
 *
 * The static methods are the logic (VisionPickupAction calls chooseBest directly).
 * As an Action it reads SharedData once, picks, and publishes SharedData.setTargetClusterXY (null = nothing worth it).
 */
public class ClusterDetectionAction extends Action {
    /** Balls closer than this chain into one group, mm. Same as RedDecideAuto LOAD_GROUP_MM. */
    public static double GROUP_MM = 250;
    /** A group's middle is only driven to when a ball is this close to it (else aim at the ball nearest the middle), mm. */
    public static double NEAR_CENTER_MM = 100;
    /** GUESS (same as RedDecideAuto): drive speed used to turn distance into time. */
    public static double SPEED_MM_PER_SECOND = 1000;
    /** GUESS (same as RedDecideAuto): slowing down, picking up and speeding up again at one stop. */
    public static double STOP_SECONDS = 1.0;

    private final AllianceColor allianceColor;
    private List<BallInformation> rawDetections; // null = read SharedData on the first update
    private Point robotPosition;                  // null = read odometry on the first update
    private int room = BallCluster.INTAKE_CAPACITY;
    private BallCluster bestCluster = null;

    /** Reads the balls (SharedData.getBallInformation) and the robot pose on its first update. */
    public ClusterDetectionAction(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public ClusterDetectionAction(List<BallInformation> rawDetections, Point robotPosition, AllianceColor allianceColor) {
        this.rawDetections = rawDetections != null ? rawDetections : new ArrayList<>();
        this.robotPosition = robotPosition;
        this.allianceColor = allianceColor;
    }

    /** How many more balls the intake can take (default: a full intake's worth). */
    public void setRoom(int room) {
        this.room = room;
    }

    @Override
    protected void update() {
        if (rawDetections == null) {
            rawDetections = SharedData.getBallInformation();
        }
        if (robotPosition == null) {
            robotPosition = SharedData.getOdometryWheelIMUPosition().toPoint();
        }
        bestCluster = chooseBest(rawDetections, robotPosition, allianceColor, room);
        SharedData.setTargetClusterXY(bestCluster == null ? null : aimPointOf(bestCluster));
        isDone = true;
    }

    public BallCluster getBestCluster() {
        return bestCluster;
    }

    public boolean isFinished() {
        return isDone;
    }

    // ------------------------------------------------------------------------------------------
    // The logic (pure: no hardware, no SharedData)
    // ------------------------------------------------------------------------------------------

    /** The group to drive to next, or null when nothing is worth it (no balls, no room, or every group has an opponent ball). */
    public static BallCluster chooseBest(List<BallInformation> balls, Point robot, AllianceColor alliance, int room) {
        if (room <= 0) {
            return null;
        }
        List<BallInformation> ours = new ArrayList<>();
        for (BallInformation ball : balls) {
            if (ball != null && ball.x < FieldConfig.MIDLINE_X) { // ponytail: AUTO-only side rule; pass a flag if TeleOp ever uses this
                ours.add(ball);
            }
        }
        List<BallCluster> groups = group(ours, alliance);

        BallCluster best = null;
        double bestScore = 0;
        for (BallCluster first : groups) {
            double score = loadScore(robot, first, null, alliance, room);
            if (first.size() < room) {
                for (BallCluster second : groups) {
                    if (second != first) {
                        score = Math.max(score, loadScore(robot, first, second, alliance, room));
                    }
                }
            }
            if (score > bestScore) {
                bestScore = score;
                best = first;
            }
        }
        return best;
    }

    /** Balls chained into groups (a ball within GROUP_MM of any ball of a group joins it). Groups with an opponent or UNKNOWN ball are dropped. */
    public static List<BallCluster> group(List<BallInformation> balls, AllianceColor alliance) {
        List<BallCluster> groups = new ArrayList<>();
        boolean[] used = new boolean[balls.size()];
        for (int start = 0; start < balls.size(); start++) {
            if (used[start]) {
                continue;
            }
            BallInformation seed = balls.get(start);
            BallCluster group = new BallCluster(BallCluster.tileColumnOf(seed.x), BallCluster.tileRowOf(seed.y));
            List<Integer> queue = new ArrayList<>();
            queue.add(start);
            used[start] = true;
            for (int q = 0; q < queue.size(); q++) {
                BallInformation ball = balls.get(queue.get(q));
                group.addBall(ball);
                for (int other = 0; other < balls.size(); other++) {
                    if (!used[other] && ball.distanceTo(balls.get(other)) < GROUP_MM) {
                        used[other] = true;
                        queue.add(other);
                    }
                }
            }
            if (!group.isEliminated(alliance)) {
                groups.add(group);
            }
        }
        return groups;
    }

    /**
     * Where to drive for a group: its middle when a ball is near the middle, else the ball nearest the middle
     * (for scattered balls the middle is empty floor).
     */
    public static Point aimPointOf(BallCluster cluster) {
        Point center = cluster.getCenter();
        Point nearest = center;
        double nearestDistance = Double.MAX_VALUE;
        for (BallInformation ball : cluster.getBalls()) {
            double distance = Point.distance(center, ball.toPoint());
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = ball.toPoint();
            }
        }
        return (nearestDistance <= NEAR_CENTER_MM) ? center : nearest;
    }

    /** Points per second of driving to first (then second, if not null) and picking up, capped by room. */
    static double loadScore(Point robot, BallCluster first, BallCluster second, AllianceColor alliance, int room) {
        BallInformation.Type nectarType = BallCluster.allianceType(alliance);
        int nectar = first.getCount(nectarType);
        int pollen = first.getCount(BallInformation.Type.YELLOW);
        Point firstAim = aimPointOf(first);
        // ponytail: straight-line distance, not the planner's route around the Hive legs; use PathPlanning.plan if that misleads
        double seconds = Point.distance(robot, firstAim) / SPEED_MM_PER_SECOND + STOP_SECONDS;
        if (second != null) {
            nectar += second.getCount(nectarType);
            pollen += second.getCount(BallInformation.Type.YELLOW);
            seconds += Point.distance(firstAim, aimPointOf(second)) / SPEED_MM_PER_SECOND + STOP_SECONDS;
        }
        return BallCluster.points(nectar, pollen, room) / seconds;
    }
}
