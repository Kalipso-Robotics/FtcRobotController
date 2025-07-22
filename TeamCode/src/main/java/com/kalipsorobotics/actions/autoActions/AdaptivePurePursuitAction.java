package com.kalipsorobotics.actions.autoActions;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Vector;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private double pathMaxVelocity = 100;
    // If the robot overshoots or skids in curves → lower it, if the robot is slow or choppy in straightaways → raise it
    private final double K = 1000; //based on how slow you want the robot to go around turns
    // If robot cuts corners or skids → reduce K, if robot slows down too much in gentle curves → increase K
    private static final double MAX_ACCELERATION = 800; // mm/s^2
    // If the robot struggles to accelerate → lower a, if it's too conservative and slow → raise a
    private final double angleKp = 1.0;


    private double startTimeMS = System.currentTimeMillis();
    private double maxTimeOutMS = 1000000000;

    private Position lastPosition;
    private double lastMilli = 0;
    ElapsedTime timeoutTimer;

    private double xVelocity;
    private double yVelocity;
    private double thetaVelocity;

    List<Position> injectedPathPoints = new ArrayList<>();
    private int pointInject = 0;
    private int segInject = 0;
    private boolean injectDone = false;

    Path newPath = null;
    private boolean finishedCurrentLoop = false;
    private final double SMOOTHER_A = 0.25;
    private final double SMOOTHER_B = 1 - SMOOTHER_A;
    private final double SMOOTHER_TOLERANCE = 0.025;
    private double change = SMOOTHER_TOLERANCE;
    private int smootherI = 1;
    private int smootherJ = 0;
    private boolean smootherDone = false;

    private int calcPoint = -1;
    private boolean calcDistanceCurvatureVelocityDone = false;

    public AdaptivePurePursuitAction(DriveTrain driveTrain, WheelOdometry wheelOdometry) {
        this.driveTrain = driveTrain;
        this.wheelOdometry = wheelOdometry;

        this.timeoutTimer = new ElapsedTime();

        this.prevFollow = Optional.empty();

        this.dependentActions.add(new DoneStateAction());
    }

    public void addPoint(double x, double y, double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        pathPoints.add(new Position(x, y, headingRad));
    }

    @Override
    protected boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    public void setIsDone(boolean isDone) {
        this.isDone = isDone;
        if (isDone) {
            driveTrain.setPower(0);
        }
    }

//    private void targetPosition(Position target, Position currentPos) {
//        //Position currentPos = wheelOdometry.getCurrentPosition();
//        Vector currentToTarget = Vector.between(currentPos, target);
//
//        double distanceToTarget = currentToTarget.getLength();
//        double targetDirection = currentToTarget.getHeadingDirection();
//        double targetAngle = target.getTheta();
//        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());
//
//        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
//        double xError = Math.cos(directionError) * distanceToTarget;
//        double yError = Math.sin(directionError) * distanceToTarget;
//
//        double powerAngle = target.getPidAngle().getPower(angleError);
//        double powerX = target.getPidX().getPower(xError);
//        double powerY = target.getPidY().getPower(yError);
//
//        //Log.d("directionalpower", String.format("power x=%.4f, power y=%.5f, powertheta=%.6f", powerX, powerY,
//        //powerAngle));
//
//        double fLeftPower = powerX + powerY + powerAngle;
//        double bLeftPower = powerX - powerY + powerAngle;
//        double fRightPower = powerX - powerY - powerAngle;
//        double bRightPower = powerX + powerY - powerAngle;
//
//        //Log.d("PurePursuit_Log",
//        //"running " + name + "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " +
//        //bRightPower);
//
//        driveTrain.setPowerWithRangeClippingMinThreshold(fLeftPower, fRightPower, bLeftPower, bRightPower, 0.4);
////        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
//        //Log.d("purepursactionlog", "target position " + target.getX() + " " + target.getY() + " " + targetAngle);
//        prevFollow = Optional.of(target);
//    }

    private void targetPosition(Position target, Position currentPos) {
        Vector toTarget = Vector.between(currentPos, target);

        double velocity = target.getVelocity(); // from Pure Pursuit
        double curvature = target.getCurvature(); // from Pure Pursuit
        double robotAngle = currentPos.getTheta();

        double headingToTarget = toTarget.getHeadingDirection();
        double directionError = MathFunctions.angleWrapRad(headingToTarget - robotAngle);

        double vx = velocity * Math.cos(directionError); // forward component
        double vy = velocity * Math.sin(directionError); // strafe component

        // Optional blend of curvature-based and PID-based orientation correction
        double angularVelocity = curvature * velocity; // rad/s
        double angleError = MathFunctions.angleWrapRad(target.getTheta() - robotAngle);
        double rotationPower = angularVelocity + (angleKp * angleError); // e.g., angleKp = 1.0

        double fLeftPower = vx + vy + rotationPower;
        double bLeftPower = vx - vy + rotationPower;
        double fRightPower = vx - vy - rotationPower;
        double bRightPower = vx + vy - rotationPower;

        double max = Math.max(1.0, Math.max(Math.abs(fLeftPower),
                Math.max(Math.abs(bLeftPower), Math.max(Math.abs(fRightPower), Math.abs(bRightPower)))));

        fLeftPower /= max;
        bLeftPower /= max;
        fRightPower /= max;
        bRightPower /= max;

        driveTrain.setPowerWithRangeClippingMinThreshold(fLeftPower, fRightPower, bLeftPower, bRightPower, 0.4);

        prevFollow = Optional.of(target);
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
//            path = smoother(new Path(injectPoints(new Path(pathPoints))), 0.25, 0.75, 0.025); cheese
//            calculateDistanceCurvatureVelocity(path);
            startTimeMS = System.currentTimeMillis();
            hasStarted = true;
            lastPosition = new Position(SharedData.getOdometryPosition());
            timeoutTimer.reset();
        }

        if (!injectDone) {
            injectPoints(new Path(pathPoints));
            return;
        } else {
            path = new Path (injectedPathPoints);
        }

        if (injectDone && !smootherDone) {
            smoother(path);
            return;
        } else {
            path = newPath;
        }

        if (injectDone && smootherDone && !calcDistanceCurvatureVelocityDone) {
            calculateDistanceCurvatureVelocity(path);
        }

        if (injectDone && smootherDone && calcDistanceCurvatureVelocityDone) {
            currentPosition = new Position(SharedData.getOdometryPosition());

            double elapsedTime = System.currentTimeMillis() - startTimeMS;

            if (elapsedTime >= maxTimeOutMS) {
                setIsDone(true);
                //Log.d("purepursaction_debug_follow", "done timeout  " + getName());
                return;
            }

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

            xVelocity = (Math.abs(lastPosition.getX() - currentPosition.getX())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
            yVelocity = (Math.abs(lastPosition.getY() - currentPosition.getY())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
            thetaVelocity = (Math.abs(lastPosition.getTheta() - currentPosition.getTheta())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));


            if(xVelocity < 0.01 && yVelocity < 0.01 && thetaVelocity < 0.01) {
                if(timeoutTimer.milliseconds() > 1000) {
                    finishedMoving();
                }
            } else {
                timeoutTimer.reset();
            }

            lastMilli = timeoutTimer.milliseconds();
            lastPosition = currentPosition;
        }

        Log.d("adaptive pure pursuit", "inject done: " + injectDone);
        Log.d("adaptive pure pursuit", "smoother done: " + smootherDone);
        Log.d("adaptive pure pursuit", "velocity done: " + calcDistanceCurvatureVelocityDone);

    }

    public void finishedMoving() {
        driveTrain.setPower(0);
        isDone = true;
    }

//    private List<Position> injectPoints(Path path) {
//        int spacingMM = 150;
//        List<Position> injectedPathPoints = new ArrayList<>();
//
//        for (int seg = 0; seg < path.numSegments(); seg++) {
//            Vector vector = path.getSegment(seg).getVector();
//            double segmentLength = vector.getLength();
//            double numPointsFit = Math.ceil(segmentLength / spacingMM);
//
//            Vector norm = vector.normalize();
//            Vector unitVector = new Vector(norm.getX() * spacingMM, norm.getY() * spacingMM);
//
//            Position start = path.getSegment(seg).getStart();
//            Position end = path.getSegment(seg).getFinish();
//
//            double startTheta = start.getTheta();
//            double endTheta = end.getTheta();
//
//            for (int i = 0; i < numPointsFit; i++) {
//                double x = start.getX() + unitVector.getX() * i;
//                double y = start.getY() + unitVector.getY() * i;
//
//                double t = i / numPointsFit;
//                double theta = MathFunctions.interpolateAngle(startTheta, endTheta, t);
//
//                injectedPathPoints.add(new Position(x, y, theta));
//            }
//        }
//
//        injectedPathPoints.add(path.getLastPoint());
//
//        return injectedPathPoints;
//    }

    private void injectPoints(Path path) {
        int spacingMM = 150;

        if (segInject == 0 && pointInject == 0 && injectedPathPoints.isEmpty()) {
            // Add the very first point from the original path only once
            injectedPathPoints.add(path.getPoint(0));
        }

        if (segInject < path.numSegments()) {
            Vector vector = path.getSegment(segInject).getVector();
            double segmentLength = vector.getLength();
            double numPointsFit = Math.ceil(segmentLength / spacingMM);

            Vector norm = vector.normalize();
            Vector unitVector = new Vector(norm.getX() * spacingMM, norm.getY() * spacingMM);

            Position start = path.getSegment(segInject).getStart();
            Position end = path.getSegment(segInject).getFinish();

            double startTheta = start.getTheta();
            double endTheta = end.getTheta();

            if (pointInject < numPointsFit) {
                double x = start.getX() + unitVector.getX() * pointInject;
                double y = start.getY() + unitVector.getY() * pointInject;

                double t = pointInject / numPointsFit;
                double theta = MathFunctions.interpolateAngle(startTheta, endTheta, t);

                injectedPathPoints.add(new Position(x, y, theta));

                pointInject++;
            } else {
                pointInject = 0;
                segInject++;
            }

        } else {
            injectDone = true;

            injectedPathPoints.add(path.getLastPoint());
        }
    }

//    private Path smoother(Path path, double a, double b, double tolerance) {
//        Path newPath = path;
//
//        double change = tolerance;
//
//        while (change >= tolerance) {
//            change  = 0.0;
//            for (int i=0; i<=path.numPoints()-1; i++) {
//                for (int j=0; j<=1; j++) {
//                    if (j==0) {
//                        double aux = newPath.getPoint(i).getX();
//                        newPath.getPoint(i).addX(a * (path.getPoint(i).getX() - newPath.getPoint(i).getX()) + b * (newPath.getPoint(i-1).getX() + newPath.getPoint(i+1).getX() - (2.0 * newPath.getPoint(i).getX())));
//                        change += Math.abs(aux - newPath.getPoint(i).getX());
//                    } else {
//                        double aux = newPath.getPoint(i).getY();
//                        newPath.getPoint(i).addY(a * (path.getPoint(i).getY() - newPath.getPoint(i).getY()) + b * (newPath.getPoint(i-1).getY() + newPath.getPoint(i+1).getY() - (2.0 * newPath.getPoint(i).getY())));
//                        change += Math.abs(aux - newPath.getPoint(i).getY());
//                    }
//                }
//            }
//        }
//
//        return newPath;
//
//    }

    private void smoother(Path path) {

        if (newPath == null && injectDone) {
            newPath = path;
        }

        if (injectDone && (!finishedCurrentLoop || change >= SMOOTHER_TOLERANCE)) {

            if (finishedCurrentLoop && change >= SMOOTHER_TOLERANCE) {
                smootherI = 1; //skip 0 we don't want to smooth first point
                change = 0.0;
                finishedCurrentLoop = false;
            }

            if (smootherI < path.numPoints()-1) { //skip last point

                if (smootherJ <= 1) {
                    if (smootherJ == 0) {
                        double aux = newPath.getPoint(smootherI).getX();
                        newPath.getPoint(smootherI).addX(SMOOTHER_A * (path.getPoint(smootherI).getX() - newPath.getPoint(smootherI).getX()) + SMOOTHER_B * (newPath.getPoint(smootherI-1).getX() + newPath.getPoint(smootherI+1).getX() - (2.0 * newPath.getPoint(smootherI).getX())));
                        change += Math.abs(aux - newPath.getPoint(smootherI).getX());
                    } else {
                        double aux = newPath.getPoint(smootherI).getY();
                        newPath.getPoint(smootherI).addY(SMOOTHER_A * (path.getPoint(smootherI).getY() - newPath.getPoint(smootherI).getY()) + SMOOTHER_B * (newPath.getPoint(smootherI-1).getY() + newPath.getPoint(smootherI+1).getY() - (2.0 * newPath.getPoint(smootherI).getY())));
                        change += Math.abs(aux - newPath.getPoint(smootherI).getY());
                    }
                    smootherJ++;
                } else {
                    smootherJ = 0;
                    smootherI++;
                }
            }

            if (smootherI == path.numPoints()-1) {
                finishedCurrentLoop = true;
            }

        } else {
            smootherDone = true;
        }
    }

    private void calculateDistanceCurvatureVelocity(Path path) {
        if (calcPoint < 0 && !calcDistanceCurvatureVelocityDone) {
            calcPoint = path.numPoints()-1;
            Log.d("adaptive pure pursuit velocity", "set calcpoint to last point: " + calcPoint);
        }

        if (calcPoint>=0) {
            if (calcPoint == 0) {
                path.getPoint(calcPoint).setDistanceAlongPath(0);
                path.getPoint(calcPoint).setCurvature(curvature(path, calcPoint));
                path.getPoint(calcPoint).setVelocity(0);

                calcDistanceCurvatureVelocityDone = true;
            } else {
                Vector vector = Vector.between(path.getPoint(calcPoint - 1), path.getPoint(calcPoint));
                path.getPoint(calcPoint).setDistanceAlongPath(path.getPoint(calcPoint - 1).getDistanceAlongPath() + vector.getLength());

                path.getPoint(calcPoint).setCurvature(curvature(path, calcPoint));

                path.getPoint(calcPoint).setVelocity(getTargetVelocity(path, calcPoint));
            }

            Log.d("adaptive pure pursuit velocity", "calculated point: " + calcPoint);
            calcPoint--;

        } else {
            calcDistanceCurvatureVelocityDone = true;
        }
    }

    public static double curvature(Path path, int positionIndex) {
//        if (positionIndex <= 0 || positionIndex >= path.numPoints() - 1) {
//            // For the first and last points, curvature is typically considered 0,
//            // or a default value, as it's a point, not a curve segment defined by three points.
//            // In a path, the ends are often straight or have specific handling.
//            return 0.0;
//        }
//
//        double x_1 = path.getPoint(positionIndex-1).getX();
//        double y_1 = path.getPoint(positionIndex-1).getY();
//        double x_2 = path.getPoint(positionIndex).getX();
//        double y_2 = path.getPoint(positionIndex).getY();
//        double x_3 = path.getPoint(positionIndex+1).getX();
//        double y_3 = path.getPoint(positionIndex+1).getY();
//
//
//        double k_1 = 0.5 * (MathFunctions.square(x_1) + MathFunctions.square(y_1) - MathFunctions.square(x_2) - MathFunctions.square(y_2)) / ((x_1 + 0.001) - x_2);
//        double k_2 = (y_1 - y_2) / (x_1 - x_2);
//        double b = 0.5 * (MathFunctions.square(x_2) - (2 * x_2 * k_1) + MathFunctions.square(y_2) - MathFunctions.square(x_3) + (2 * x_3 * k_1) - MathFunctions.square(y_3)) / ((x_3 * k_2) - y_3 + y_2 - (x_2 * k_2));
//        double a = k_1 - (k_2 * b);
//        double r = Math.sqrt(MathFunctions.square(x_1 - a) + MathFunctions.square(y_1 - b));
//
//        return 1/r;

        // Ensure that positionIndex allows for accessing points before and after
        if (positionIndex <= 0 || positionIndex >= path.numPoints() - 1) {
            // For the first and last points, curvature is typically considered 0,
            // or a default value, as it's a point, not a curve segment defined by three points.
            // In a path, the ends are often straight or have specific handling.
            return 0.0;
        }

        Position p1 = path.getPoint(positionIndex - 1);
        Position p2 = path.getPoint(positionIndex);
        Position p3 = path.getPoint(positionIndex + 1);

        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        double x3 = p3.getX();
        double y3 = p3.getY();

        // Calculate the signed area (twice the area of the triangle formed by the three points)
        // This is a robust way to check for collinearity and determine the sign of curvature.
        double area2 = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);

        // Check for collinearity (points lie on a straight line)
        // Use a small epsilon for floating point comparison
        final double EPSILON = 1e-9; // A very small number to account for floating-point inaccuracies
        if (Math.abs(area2) < EPSILON) {
            return 0.0; // Points are collinear, so curvature is zero
        }

        // Calculate the lengths of the sides of the triangle
        double a = Math.sqrt(MathFunctions.square(x1 - x2) + MathFunctions.square(y1 - y2)); // Distance p1 to p2
        double b = Math.sqrt(MathFunctions.square(x2 - x3) + MathFunctions.square(y2 - y3)); // Distance p2 to p3
        double c = Math.sqrt(MathFunctions.square(x3 - x1) + MathFunctions.square(y3 - y1)); // Distance p1 to p3

        // Calculate the radius of the circumcircle of the triangle formed by p1, p2, p3
        // R = (abc) / (4 * Area)
        double radius = (a * b * c) / (2 * Math.abs(area2)); // Use Math.abs(area2) since radius is always positive

        // If radius is extremely large (effectively infinite), curvature is 0
        if (radius > 1e9) { // Arbitrarily large number, adjust as needed
            return 0.0;
        }

        // Curvature is 1/R. The sign of the curvature indicates the direction of the curve.
        // A positive curvature can indicate a turn to the left, and negative to the right,
        // depending on your coordinate system and how "area2" is interpreted.
        // For pure pursuit, often the magnitude is used for speed, and steering is determined
        // by the look-ahead point's position relative to the robot's heading.
        // However, if you want signed curvature, you can divide area2 by the radius components.
        // For general usage in path following, a simple 1/R is often sufficient, with the controller
        // deciding the turn direction. If the original intent was signed curvature:
        return area2 / (radius * radius * 2); // This gives signed curvature more robustly.
        // If your system only needs positive curvature, use Math.abs(1/radius)
        // or just 1/radius if radius is always positive.

        // The formula for signed curvature k = 2 * (x1(y2-y3) + x2(y3-y1) + x3(y1-y2)) / ((x1-x2)^2 + (y1-y2)^2) * ((x2-x3)^2 + (y2-y3)^2) * ((x3-x1)^2 + (y3-y1)^2))
        // is more complex and not always needed if the pure pursuit logic handles turn direction.

    }

    private double calculateVelocity(Path path, int positionIndex) {
        if (positionIndex <= 0 || positionIndex >= path.numPoints() - 1) {
            return 0;
        }

        double curvature = Math.abs(curvature(path, positionIndex));
        if (curvature < 1e-6) return pathMaxVelocity; // straight line
        return Math.min(pathMaxVelocity, K / curvature);
    }

    private double getTargetVelocity(Path path, int positionIndex) {
        double v_i = path.getPoint(positionIndex-1).getVelocity();
        double a = MAX_ACCELERATION;
        double d = path.getPoint(positionIndex).getDistanceAlongPath() - path.getPoint(positionIndex-1).getDistanceAlongPath();

        double v_f = Math.sqrt(MathFunctions.square(v_i) + (2 * a * d));

        return Math.min(v_f, calculateVelocity(path, positionIndex));
    }
}
