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

    private double pathMaxVelocity = 100; //todo figuer this out
    private final double K = 1; // 1-5, based on how slow you want the robot to go around turns

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
            path = smoother(new Path(injectPoints(new Path(pathPoints))), 0.25, 0.75, 0.025);
            calculateDistanceCurvatureVelocity(path);
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

    private List<Position> injectPoints(Path path) {
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

        return injectedPathPoints;
    }

    private Path smoother(Path path, double a, double b, double tolerance) {

        Path newPath = path;

        double change = tolerance;

        while (change >= tolerance) {
            change  = 0.0;
            for (int i=0; i<=path.numPoints()-1; i++) {
                for (int j=0; j<=1; j++) {
                    if (j==0) {
                        double aux = newPath.getPoint(i).getX();
                        newPath.getPoint(i).addX(a * (path.getPoint(i).getX() - newPath.getPoint(i).getX()) + b * (newPath.getPoint(i-1).getX() + newPath.getPoint(i+1).getX() - (2.0 * newPath.getPoint(i).getX())));
                        change += Math.abs(aux - newPath.getPoint(i).getX());
                    } else {
                        double aux = newPath.getPoint(i).getY();
                        newPath.getPoint(i).addY(a * (path.getPoint(i).getY() - newPath.getPoint(i).getY()) + b * (newPath.getPoint(i-1).getY() + newPath.getPoint(i+1).getY() - (2.0 * newPath.getPoint(i).getY())));
                        change += Math.abs(aux - newPath.getPoint(i).getY());
                    }
                }
            }
        }

        return newPath;

    }

    private void calculateDistanceCurvatureVelocity(Path path) {
        for (int i=path.numPoints()-1; i>=0; i--) { //todo get rid of for loops

            Vector vector = Vector.between(path.getPoint(i-1), path.getPoint(i));
            path.getPoint(i).setDistanceAlongPath(path.getPoint(i-1).getDistanceAlongPath() + vector.getLength());

            path.getPoint(i).setCurvature(curvature(path, i));

            path.getPoint(i).setVelocity(getTargetVelocity(path, i));

            if (i==0) {
                path.getPoint(i).setDistanceAlongPath(0);

                path.getPoint(i).setCurvature(0);

                path.getPoint(i).setVelocity(0);
            } else if (i == path.numPoints()-1) {
                path.getPoint(i).setCurvature(0);
            }

        }
    }

    public static double curvature(Path path, int positionIndex) {
        double x_1 = path.getPoint(positionIndex-1).getX();
        double y_1 = path.getPoint(positionIndex-1).getY();
        double x_2 = path.getPoint(positionIndex).getX();
        double y_2 = path.getPoint(positionIndex).getY();
        double x_3 = path.getPoint(positionIndex+1).getX();
        double y_3 = path.getPoint(positionIndex+1).getY();


        double k_1 = 0.5 * (MathFunctions.square(x_1) + MathFunctions.square(y_1) - MathFunctions.square(x_2) - MathFunctions.square(y_2)) / ((x_1 + 0.001) - x_2);
        double k_2 = (y_1 - y_2) / (x_1 - x_2);
        double b = 0.5 * (MathFunctions.square(x_2) - (2 * x_2 * k_1) + MathFunctions.square(y_2) - MathFunctions.square(x_3) + (2 * x_3 * k_1) - MathFunctions.square(y_3)) / ((x_3 * k_2) - y_3 + y_2 - (x_2 * k_2));
        double a = k_1 - (k_2 * b);
        double r = Math.sqrt(MathFunctions.square(x_1 - a) + MathFunctions.square(y_1 - b));

        return 1/r;
    }

    private double calculateVelocity(Path path, int positionIndex) {
        return Math.min(this.pathMaxVelocity, (this.K / curvature(path, positionIndex)));
    }

    private double getTargetVelocity(Path path, int positionIndex) {
        double v_i = path.getPoint(positionIndex-1).getVelocity();
        double a = 100; // todo max acceleration figure out
        double d = path.getPoint(positionIndex).getDistanceAlongPath() - path.getPoint(positionIndex-1).getDistanceAlongPath();

        double v_f = Math.sqrt(MathFunctions.square(v_i) + (2 * a * d));

        return Math.min(v_f, calculateVelocity(path, positionIndex));
    }
}
