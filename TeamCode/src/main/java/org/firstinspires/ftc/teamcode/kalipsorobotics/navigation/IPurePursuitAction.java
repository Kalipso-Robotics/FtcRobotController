package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

import java.util.List;

public abstract class IPurePursuitAction extends Action {
    double DEFAULT_P_XY = 1.0 / 350.0;
    double DEFAULT_P_ANGLE = 1.0 / Math.toRadians(90);


    public void addPoint(double x, double y, double headingDeg) {
        getPathPoints().add(
                new Position(x, y, Math.toRadians(headingDeg), DEFAULT_P_XY, DEFAULT_P_ANGLE)
        );
    }

    public void addPoint(double x, double y, double headingDeg, double pXY, double pAngle) {
        getPathPoints().add(
                new Position(x, y, Math.toRadians(headingDeg), pXY, pAngle)
        );
    }

    public void clearPoints() {
        getPathPoints().clear();
    }
    public void setLookAheadRadius(double radiusMM) {};
    public void setFinalSearchRadiusMM(double searchRadiusMM) {};
    public void setMaxTimeOutMS(double maxTimeOutMS) {};
    public void setFinalAngleLockingThresholdDeg(double deg) {};
    public void setPathAngleToleranceDeg(double deg) {};
    public abstract List<Position> getPathPoints();
}
