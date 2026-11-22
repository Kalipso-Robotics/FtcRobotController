package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

/**
 * The two Flowers on our side (own frame, mm).
 * Setup point: we drive here first so we can intake the Flower straight on.
 */
public enum Flower {
    // TODO-measure: setup point/heading, heading at the flower, exit point/heading ((0, 0) = placeholder;
    // FullFlowerAction skips the trip (logs an error) while setup or exit is (0, 0), so the robot never drives into the corner).
    //   setup point   = where the robot lines up in front of the flower before driving in, intake side facing it
    //   flower heading= robot heading while sitting on the flower pile (intake side toward the pile)
    //   exit point    = where it backs out to afterwards, clear of the flower, before driving to shoot
    // Lower timeout: how long FlowerLower keeps retrying while the intake pivot is still coming down
    // (IntakeConfig.PIVOT_MOVE_MS = 400 plus margin), not a measured value.
    FLOWER_1(new Point(1196, 62),   // x wall
            new Point(0, 0), 0,     // setup point, setup heading
            0,                      // heading at the flower
            new Point(0, 0), 0,     // exit point, exit heading
            1000),                  // flower lower timeout ms
    FLOWER_2(new Point(64, 2385),   // y wall
            new Point(0, 0), 0,
            0,
            new Point(0, 0), 0,
            1000);

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
