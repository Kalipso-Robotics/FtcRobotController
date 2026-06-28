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

    public int getLastPointIndex() {
        return getPathPoints().size() - 1;
    }

    public abstract void rebuildPath();
    public abstract void setLookAheadRadius(double radiusMM);
    public abstract double getLastSearchRadiusMM();
    public abstract void setFinalSearchRadiusMM(double searchRadiusMM);
    public abstract void setMaxTimeOutMS(double maxTimeOutMS);
    public abstract void setFinalAngleLockingThresholdDeg(double deg);
    public abstract boolean isWithinDistancePoint(int index, double distanceThreshold);
    public abstract void setPathAngleToleranceDeg(double pathAngleToleranceDeg);
    public abstract void setEnablePowerScalingForPath(boolean enablePowerScalingForPath);

    public abstract List<Position> getPathPoints();

}
