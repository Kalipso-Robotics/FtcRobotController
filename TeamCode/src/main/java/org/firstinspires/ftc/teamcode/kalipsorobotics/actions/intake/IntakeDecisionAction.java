package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootDecisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Decides which place to intake from next by looking TWO trips ahead.
 *
 * For every pair of places (first, second) it adds up the whole route:
 *   robot -> first place -> shoot point -> second place -> shoot point
 * and keeps the pair with the SHORTEST route. We only go to the first place now;
 * the second one gets decided again next trip (things may have changed by then).
 *
 * A place is skipped if its IntakeSources number is 0 (no balls there).
 * Tip zones are ignored until the flowers and the Garden are all empty
 * (their balls are sure and easy to pick up; tip zone balls are not).
 * If only one place has balls, the route is just: robot -> that place -> shoot point.
 *
 * The shoot point comes from ShootDecisionAction (the side of the Cell that is up).
 */
public class IntakeDecisionAction extends Action {

    // Where we stand to intake each tip zone. Position angles are in radians.
    public static final Position TIP_ZONE_1_POSITION = new Position(424, 2224, Math.toRadians(45));
    public static final Position TIP_ZONE_2_POSITION = new Position(424, 1404, Math.toRadians(305));

    private final ShootDecisionAction shootDecision;   // tells us where we shoot from
    private IntakeSources chosenSource = null;         // the answer (null = nothing left to intake)

    public IntakeDecisionAction(ShootDecisionAction shootDecision) {
        this.shootDecision = shootDecision;
        this.name = "IntakeDecisionAction";
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        decide();
        isDone = true;
    }

    /** Picks where to intake. Call this once at the start of every trip. */
    public void decide() {
        Point robot = SharedData.getOdometryWheelIMUPosition().toPoint();
        shootDecision.decide();
        Point shoot = shootDecision.getShootPosition().toPoint();

        chosenSource = null;
        double shortestRoute = Double.MAX_VALUE;

        for (IntakeSources first : IntakeSources.values()) {
            if (!canGoTo(first)) {
                continue;
            }

            // trip 1: robot -> first place -> shoot
            double firstTrip = distanceTo(robot, pointOf(first)) + distanceTo(pointOf(first), shoot);
            boolean foundSecondPlace = false;

            for (IntakeSources second : IntakeSources.values()) {
                if (second == first || !canGoTo(second)) {
                    continue;
                }
                foundSecondPlace = true;

                // trip 2: shoot -> second place -> shoot
                double secondTrip = distanceTo(shoot, pointOf(second)) + distanceTo(pointOf(second), shoot);

                double route = firstTrip + secondTrip;
                if (route < shortestRoute) {
                    shortestRoute = route;
                    chosenSource = first;
                }
            }

            // Only one place has balls: the route is just trip 1
            if (!foundSecondPlace && firstTrip < shortestRoute) {
                shortestRoute = firstTrip;
                chosenSource = first;
            }
        }
    }

    /** The distance formula: sqrt((x2 - x1)^2 + (y2 - y1)^2). */
    public static double distanceTo(Point from, Point to) {
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private boolean hasBalls(IntakeSources source) {
        return source.getIntakeValue() > 0;
    }

    private boolean isTipZone(IntakeSources source) {
        return source == IntakeSources.TIP_ZONE_1 || source == IntakeSources.TIP_ZONE_2;
    }

    /** True while a flower or the Garden still has balls. */
    private boolean surePlaceHasBalls() {
        return hasBalls(IntakeSources.FLOWER_1) || hasBalls(IntakeSources.FLOWER_2) || hasBalls(IntakeSources.GARDEN);
    }

    /** True if we may go to this place: it has balls, and it is not a tip zone while sure places are left. */
    private boolean canGoTo(IntakeSources source) {
        if (!hasBalls(source)) {
            return false;
        }
        if (isTipZone(source) && surePlaceHasBalls()) {
            return false;   // flowers and Garden first
        }
        return true;
    }

    public IntakeSources getChosenSource() {
        return chosenSource;
    }

    /** Where each place is on the field. */
    public static Point pointOf(IntakeSources source) {
        switch (source) {
            case FLOWER_1:   return Flower.FLOWER_1.getPoint();
            case FLOWER_2:   return Flower.FLOWER_2.getPoint();
            case GARDEN:     return FieldConfig.gardenPoint;
            case TIP_ZONE_1: return TIP_ZONE_1_POSITION.toPoint();
            case TIP_ZONE_2: return TIP_ZONE_2_POSITION.toPoint();
            default:         return null;
        }
    }

    /** Which way the robot faces (degrees) when it intakes at each place. */
    public static double headingDegOf(IntakeSources source) {
        switch (source) {
            case FLOWER_1:   return Flower.FLOWER_1.getFlowerHeadingDeg();
            case FLOWER_2:   return Flower.FLOWER_2.getFlowerHeadingDeg();
            case TIP_ZONE_1: return Math.toDegrees(TIP_ZONE_1_POSITION.getTheta());
            case TIP_ZONE_2: return Math.toDegrees(TIP_ZONE_2_POSITION.getTheta());
            default:         return 0;   // TODO: measure GARDEN heading
        }
    }
}
