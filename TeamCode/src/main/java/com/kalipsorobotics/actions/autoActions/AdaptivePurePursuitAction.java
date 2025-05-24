package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Vector;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.SharedData;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AdaptivePurePursuitAction extends Action {

    DriveTrain driveTrain;
    WheelOdometry wheelOdometry;

    List<Position> pathPoints = new ArrayList<Position>();

    Path path;
    static final private double LOOK_AHEAD_RADIUS_MM = 75;

    private double currentLookAheadRadius;
    static final private double LAST_RADIUS_MM = 15;

    private Position currentPosition = new Position(SharedData.getOdometryPosition());

    Optional<Position> follow;
    Optional<Position> prevFollow;

    private double lastSearchRadius = LAST_RADIUS_MM;

    private double finalAngleLockingThreshholdDeg = 1.5;

    public AdaptivePurePursuitAction(DriveTrain driveTrain, WheelOdometry wheelOdometry) {
        this.driveTrain = driveTrain;
        this.wheelOdometry = wheelOdometry;
    }

    public void addPoint(double x, double y, double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        pathPoints.add(new Position(x, y, headingRad));
    }

    @Override
    protected boolean checkDoneCondition() {
        return false;
    }

    @Override
    public void setIsDone(boolean isDone) {
        this.isDone = isDone;
        if (isDone) {
            driveTrain.setPower(0);
        }
    }

    private void targetPosition(Position target, Position currentPos) {
        //Position currentPos = wheelOdometry.getCurrentPosition();
        Vector currentToTarget = Vector.between(currentPos, target);

        double distanceToTarget = currentToTarget.getLength();
        double targetDirection = currentToTarget.getHeadingDirection();
        double targetAngle = target.getTheta();
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());

        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double xError = Math.cos(directionError) * distanceToTarget;
        double yError = Math.sin(directionError) * distanceToTarget;

        double powerAngle = target.getPidAngle().getPower(angleError);
        double powerX = target.getPidX().getPower(xError);
        double powerY = target.getPidY().getPower(yError);

        //Log.d("directionalpower", String.format("power x=%.4f, power y=%.5f, powertheta=%.6f", powerX, powerY,
        //powerAngle));

        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;
        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;

        //Log.d("PurePursuit_Log",
        //"running " + name + "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " +
        //bRightPower);

        driveTrain.setPowerWithRangeClippingMinThreshold(fLeftPower, fRightPower, bLeftPower, bRightPower, 0.4);
//        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
        //Log.d("purepursactionlog", "target position " + target.getX() + " " + target.getY() + " " + targetAngle);
        prevFollow = Optional.of(target);
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            path = new Path(pathPoints);
            hasStarted = true;
        }

        currentPosition = new Position(SharedData.getOdometryPosition());

        currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM;

        Position lastPoint = path.getLastPoint();

        if (prevFollow.isPresent() && (path.findIndex(prevFollow.get()) > (path.numPoints() - 2))) {
            currentLookAheadRadius = lastSearchRadius;
        }

        follow = path.lookAhead(currentPosition, prevFollow, currentLookAheadRadius);

        if (follow.isPresent()) {
            targetPosition(follow.get(), currentPosition);
        } else {
            if (Math.abs(lastPoint.getTheta() - currentPosition.getTheta()) <= Math.toRadians(finalAngleLockingThreshholdDeg) ) {
                finishedMoving();
            } else {
                targetPosition(lastPoint, currentPosition);
            }
        }
    }

    public void finishedMoving() {
        driveTrain.setPower(0);
    }

    public void injectPoints(Path path) {
        int spacingMM = 150;
        List<Position> injectedPathPoints = new ArrayList<Position>();

        for(int i = 0; i<path.numSegments(); i++) {
            Vector vector = path.getSegment(i).getVector();
            double numPointsFit = Math.ceil(vector.getLength() / spacingMM);
            vector = new Vector(vector.normalize().getX() * spacingMM, vector.normalize().getY() * spacingMM);
            for (i=0; i<numPointsFit; i++) {
                injectedPathPoints.add(new Position(path.getSegment(i).getStart().getX() + vector.getX() * i, path.getSegment(i).getStart().getY() + vector.getY() * i, 0));
            }
        }
        injectedPathPoints.add(path.getLastPoint());
    }

}
