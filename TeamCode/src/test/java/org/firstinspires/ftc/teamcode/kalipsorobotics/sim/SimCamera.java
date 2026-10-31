package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * A fake camera for the sim. The real chain is detector -> Raytracer -> SharedData.setBallInformation(...);
 * the sim has no camera image, so this publishes the same list straight from where the balls really are.
 *
 * What it "sees": loose balls lying on the floor, in front of the robot, inside the field of view and between a
 * minimum and a maximum distance. Each ball is missed with a small chance every frame, and the position it reports is
 * off by a small random amount. The random numbers come from a fixed seed, so a run stays the same every time.
 *
 * UNMEASURED: every number here is a guess until it is fitted to the real camera.
 */
final class SimCamera {

    /** On the Setup screen. */
    static boolean on = true;
    /** The diagonal field of view of the image, in degrees. */
    static double fovDiagonalDeg = 100;
    /** Image shape. The VisionConfig.ARDUCAM stream is 1280x720. */
    static final double IMAGE_WIDTH = 1280, IMAGE_HEIGHT = 720;
    /** Farthest and nearest distance (from the robot's centre) at which a ball is seen. The floor just in front is below the lens. */
    static double maxRangeMm = 2000;
    static double minRangeMm = 300;
    /** Chance, in percent, that one ball is missed in one frame. */
    static double missPercent = 2;
    /** A seen ball is reported off by up to this much in x and in y. */
    static double errorMm = 20;

    /** Time between frames. */
    static final long FRAME_NANOS = 33_000_000L;

    private static final double POLLEN_RADIUS_MM = 35.56;
    private static final double NECTAR_RADIUS_MM = 45.72;
    /** A ball whose centre is this much above its own radius is not lying on the floor (it is in a Flower or a Cell). */
    private static final double ON_FLOOR_TOLERANCE_MM = 25;

    private final Random random = new Random(12345);

    /** The half-angle (radians) of the view to each side of straight ahead, from the diagonal field of view of a 16:9 image. */
    static double halfWidthRad() {
        double tanHalfDiagonal = Math.tan(Math.toRadians(fovDiagonalDeg) / 2);
        double widthShare = IMAGE_WIDTH / Math.hypot(IMAGE_WIDTH, IMAGE_HEIGHT);
        return Math.atan(tanHalfDiagonal * widthShare);
    }

    /**
     * One frame.
     *
     * @param balls  the world's balls as {id, type (0 pollen, 1 red nectar, 2 blue nectar), x, y, z, held}, in mm
     * @param robotX,robotY,robotTheta where the robot really is (mm, mm, radians; theta is where the front points)
     * @return the balls the camera reports, in the field frame
     */
    List<BallInformation> see(List<double[]> balls, double robotX, double robotY, double robotTheta) {
        List<BallInformation> seen = new ArrayList<>();
        double halfWidth = halfWidthRad();
        for (double[] ball : balls) {
            int type = (int) ball[1];
            double radius = (type == 0) ? POLLEN_RADIUS_MM : NECTAR_RADIUS_MM;
            boolean held = ball[5] != 0;
            boolean onFloor = ball[4] <= radius + ON_FLOOR_TOLERANCE_MM;
            if (held || !onFloor) {
                continue;
            }
            double dx = ball[2] - robotX;
            double dy = ball[3] - robotY;
            double distance = Math.hypot(dx, dy);
            if (distance < minRangeMm || distance > maxRangeMm) {
                continue;
            }
            double bearing = normalize(Math.atan2(dy, dx) - robotTheta);
            if (Math.abs(bearing) > halfWidth) {
                continue;
            }
            // Always draw the same three numbers for a ball in view, so a run does not depend on which ball is missed.
            boolean missed = random.nextDouble() * 100 < missPercent;
            double errorX = (random.nextDouble() * 2 - 1) * errorMm;
            double errorY = (random.nextDouble() * 2 - 1) * errorMm;
            if (missed) {
                continue;
            }
            seen.add(new BallInformation(ball[2] + errorX, ball[3] + errorY, typeOf(type), 1f, distance));
        }
        return seen;
    }

    /** One frame, published to SharedData the way the Raytracer does it. */
    void publish(SimWorld world) {
        List<BallInformation> seen = see(world.ballPositions(), world.getX(), world.getY(), world.getTheta());
        SharedData.setBallInformation(seen);
    }

    private static BallInformation.Type typeOf(int type) {
        if (type == 1) {
            return BallInformation.Type.RED;
        }
        if (type == 2) {
            return BallInformation.Type.BLUE;
        }
        return BallInformation.Type.YELLOW;
    }

    private static double normalize(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
