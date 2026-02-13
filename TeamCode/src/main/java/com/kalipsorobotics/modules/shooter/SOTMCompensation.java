package com.kalipsorobotics.modules.shooter;

import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;
import com.kalipsorobotics.utilities.KLog;

public class SOTMCompensation {

    public static class SOTMResult {
        private double distance;
        private double targetAngleRad;
        public SOTMResult(double distance, double targetAngleRad) {
           this.distance = distance;
           this.targetAngleRad = targetAngleRad;
        }

        public double getDistance() {
            return distance;
        }

        public double getTargetAngleRad() {
            return targetAngleRad;
        }

        @Override
        public String toString() {
            return "SOTMResult(" +
                    "distance =" + distance +
                    ", targetAngleRad =" + targetAngleRad +
                    ')';
        }
    }

    public static SOTMResult calculateCompensation(Point targetPoint, Position currentPos, Velocity robotVelocity) {
        double distance = currentPos.toPoint().distanceTo(targetPoint);
        double flightTimeMS = FlightTimeInterpolationLUT.calculate(distance);

        double deltaX = targetPoint.getX() - currentPos.getX();
        double deltaY = targetPoint.getY() - currentPos.getY();

        double desiredXVelocity = deltaX / flightTimeMS;
        double desiredYVelocity = deltaY / flightTimeMS;
        double desiredThetaVelocity = Math.atan2(deltaY, deltaX);

        Velocity desiredVelocity = new Velocity(desiredXVelocity, desiredYVelocity, desiredThetaVelocity);

        Velocity launchVelocity = desiredVelocity.minus(robotVelocity);

        double compensatedDistance = Math.hypot(launchVelocity.getX(), launchVelocity.getY());
        double compensatedAngleRad = Math.atan2(launchVelocity.getY(), launchVelocity.getX());

        SOTMResult results = new SOTMResult(compensatedDistance, compensatedAngleRad);

        KLog.d("SOTM", "Desired Velocity: " + desiredVelocity +
                " Launch Velocity: " + launchVelocity +
                " Results: " + results +
                " Delta Distance: " + (distance - compensatedDistance)
        );

        return results;
    }
}
