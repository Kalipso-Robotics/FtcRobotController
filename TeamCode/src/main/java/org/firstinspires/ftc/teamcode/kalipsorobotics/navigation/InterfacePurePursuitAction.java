package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

import java.util.List;

public interface InterfacePurePursuitAction {
    double DEFAULT_P_XY = 1.0 / 350.0;
    double DEFAULT_P_ANGLE = 1.0 / Math.toRadians(90);

    List<Position> getPathPoints();

    default void addPoint(double x, double y, double headingDeg) {
        getPathPoints().add(
                new Position(x, y, Math.toRadians(headingDeg), DEFAULT_P_XY, DEFAULT_P_ANGLE)
        );
    }

    default void addPoint(double x, double y, double headingDeg, double pXY, double pAngle) {
        getPathPoints().add(
                new Position(x, y, Math.toRadians(headingDeg), pXY, pAngle)
        );
    }

    default void clearPoints() {
        getPathPoints().clear();
    }
    void setLookAheadRadius(double radiusMM);
    void setFinalSearchRadiusMM(double searchRadiusMM);
    void setMaxTimeOutMS(double maxTimeOutMS);
    void setFinalAngleLockingThresholdDeg(double deg);
    void setPathAngleToleranceDeg(double deg);

    void update();
}