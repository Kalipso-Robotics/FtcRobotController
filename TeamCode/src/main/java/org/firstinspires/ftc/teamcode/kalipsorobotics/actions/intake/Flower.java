package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

/**
 * The two Flowers on our side (own frame, mm).
 * Setup point: we drive here first so we can intake the Flower straight on.
 */
public enum Flower {
    // TODO: measure all values except the flower point (0 = placeholder)
    FLOWER_1(new Point(1196, 62),   // x wall
            new Point(0, 0), 0,     // setup point, setup heading
            0,                      // heading at the flower
            new Point(0, 0), 0,     // exit point, exit heading
            0),                     // flower lower timeout ms
    FLOWER_2(new Point(64, 2385),   // y wall
            new Point(0, 0), 0,
            0,
            new Point(0, 0), 0,
            0);

    private final Point point;
    private final Point setupPoint;
    private final double setupHeadingDeg;
    private final double flowerHeadingDeg;
    private final Point exitPoint;
    private final double exitHeadingDeg;
    private final double timeoutMS;

    Flower(Point point, Point setupPoint, double setupHeadingDeg, double flowerHeadingDeg,
           Point exitPoint, double exitHeadingDeg, double timeoutMS) {
        this.point = point;
        this.setupPoint = setupPoint;
        this.setupHeadingDeg = setupHeadingDeg;
        this.flowerHeadingDeg = flowerHeadingDeg;
        this.exitPoint = exitPoint;
        this.exitHeadingDeg = exitHeadingDeg;
        this.timeoutMS = timeoutMS;
    }

    public Point getPoint() { return point; }
    public Point getSetupPoint() { return setupPoint; }
    public double getSetupHeadingDeg() { return setupHeadingDeg; }
    public double getFlowerHeadingDeg() { return flowerHeadingDeg; }
    public Point getExitPoint() { return exitPoint; }
    public double getExitHeadingDeg() { return exitHeadingDeg; }
    public double getTimeoutMS() { return timeoutMS; }
}
