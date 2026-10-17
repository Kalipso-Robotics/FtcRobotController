package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;

public class BallCluster {
    public static final double TILE_SIZE_MM = 609.6;
    public static final int POLLEN_POINTS = 1;
    public static final int NECTAR_POINTS = 2;
    public static double DUPLICATE_TOLERANCE_MM = 35.5;
    public static final int INTAKE_CAPACITY = 4;

    public final int tileColumn;
    public final int tileRow;
    private final List<BallInformation> balls = new ArrayList<>();
    private final EnumMap<BallInformation.Type, Integer> counts;

    private double sumX = 0;
    private double sumY = 0;
    private Point center = new Point(0, 0);

    public BallCluster(int tileColumn, int tileRow) {
        this.tileColumn = tileColumn;
        this.tileRow = tileRow;
        this.counts = new EnumMap<>(BallInformation.Type.class);
        for (BallInformation.Type type : BallInformation.Type.values()) {
            counts.put(type, 0);
        }
    }

    /** Adds a single ball and updates running statistics in O(1) time (no loops). */
    public void addBall(BallInformation ball) {
        balls.add(ball);
        counts.put(ball.type, counts.get(ball.type) + 1);

        sumX += ball.x;
        sumY += ball.y;
        this.center = new Point(sumX / balls.size(), sumY / balls.size());
    }

    public List<BallInformation> getBalls() {
        return Collections.unmodifiableList(balls);
    }

    public Point getCenter() {
        return center;
    }

    public int getCount(BallInformation.Type type) {
        return counts.get(type);
    }

    public int size() {
        return balls.size();
    }

    public boolean containsOpponent(AllianceColor alliance) {
        return getCount(opponentType(alliance)) > 0;
    }

    public boolean isEliminated(AllianceColor alliance) {
        return containsOpponent(alliance) || getCount(BallInformation.Type.UNKNOWN) > 0;
    }

    public int getScore(AllianceColor alliance) {
        if (isEliminated(alliance)) {
            return 0;
        }
        int nectar = Math.min(getCount(allianceType(alliance)), INTAKE_CAPACITY);
        int pollen = Math.min(getCount(BallInformation.Type.YELLOW), INTAKE_CAPACITY - nectar);
        return nectar * NECTAR_POINTS + pollen * POLLEN_POINTS;
    }

    public static int tileColumnOf(double x) {
        return (int) Math.floor(x / TILE_SIZE_MM);
    }

    public static int tileRowOf(double y) {
        return (int) Math.floor(y / TILE_SIZE_MM);
    }

    public static BallInformation.Type allianceType(AllianceColor alliance) {
        return alliance == AllianceColor.RED ? BallInformation.Type.RED : BallInformation.Type.BLUE;
    }

    public static BallInformation.Type opponentType(AllianceColor alliance) {
        return alliance == AllianceColor.RED ? BallInformation.Type.BLUE : BallInformation.Type.RED;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "Tile(%d, %d) center (%.0f, %.0f) count=%d",
                tileColumn, tileRow, center.getX(), center.getY(), balls.size());
    }
}