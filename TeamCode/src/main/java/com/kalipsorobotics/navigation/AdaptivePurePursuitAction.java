package com.kalipsorobotics.navigation;

import static androidx.core.math.MathUtils.clamp;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Vector;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class AdaptivePurePursuitAction extends Action {

    DriveTrain driveTrain;
    Odometry wheelOdometry;

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

    private double pathMaxVelocity = 250;
    // If the robot overshoots or skids in curves → lower it, if the robot is slow or choppy in straightaways → raise it
    private final double K = 900; //based on how slow you want the robot to go around turns
    // If robot cuts corners or skids → reduce K, if robot slows down too much in gentle curves → increase K
    private static final double MAX_ACCELERATION = 10000; // 800 mm/s^2
    // If the robot struggles to accelerate → lower a, if it's too conservative and slow → raise a

    private double startTimeMS = System.currentTimeMillis();
    private double maxTimeOutMS = 1000000000;

    private Position lastPosition;
    private double lastMilli = 0;
    ElapsedTime timeoutTimer;

    private double currentVelocityMmPerS;
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
    private double lastUpdateTime;
    private double lastHeadingError = 0;

    private double WHEELBASE_LENGTH = 300; //front wheel to back wheel
    private double TRACK_WIDTH = 400; //side to side
    private double K_p = 0.00001; // 0.0000001
    private double K_a = 0.001; // 0.001
    private double K_v = 0.0004; // 0.0005

    public AdaptivePurePursuitAction(DriveTrain driveTrain, Odometry wheelOdometry) {
        this.driveTrain = driveTrain;
        this.wheelOdometry = wheelOdometry;

        this.timeoutTimer = new ElapsedTime();
        lastUpdateTime = timeoutTimer.milliseconds();

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
        Log.d("ppDebug", "robotAngle: " + robotAngle);

        // Robot‑relative vector to lookahead (in meters)
        double dx = target.getX() - currentPos.getX();

        double dy = target.getY() - currentPos.getY();
        // Rotate into robot frame
        double x_r =  Math.cos(robotAngle)*dx + Math.sin(robotAngle)*dy;
        Log.d("ppDebug", "x_r: " + x_r);

        double y_r = -Math.sin(robotAngle)*dx + Math.cos(robotAngle)*dy;
        // Direction and magnitude
        double distance = Math.sqrt(x_r*x_r + y_r*y_r);
        double headingError = Math.atan2(y_r, x_r);
        // Choose speeds
        double cosHE = Math.cos(headingError);
        Log.d("ppDebug", "cosHE: " + cosHE);

        double vx = velocity * cosHE;
        double vy = velocity * Math.sin(headingError);

        double angleError = MathFunctions.angleWrapRad(
                target.getTheta() - currentPos.getTheta()
        );
        double omega = 0;
        if (Math.abs(angleError) > Math.toRadians(5)) {  // only correct if >5°
            omega = target.getPidAngle().getPower(angleError);
        }

        Log.d("ppDebug", "vx: " + vx);
        Log.d("ppDebug", "vy: " + vy);


        double fLeftVelocity = vx + vy + ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;
        double bLeftVelocity = vx - vy + ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;
        double fRightVelocity = vx - vy - ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;
        double bRightVelocity = vx + vy - ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;

        Log.d("ppDebug", "fLeftVelocity: " + fLeftVelocity);

        Log.d("ppDebug", "acceleration: " + target.getAcceleration());


        double fLeftPower = calculateMotorOutput(fLeftVelocity, target.getAcceleration());
        double bLeftPower = calculateMotorOutput(bLeftVelocity, target.getAcceleration());
        double fRightPower = calculateMotorOutput(fRightVelocity, target.getAcceleration());
        double bRightPower = calculateMotorOutput(bRightVelocity, target.getAcceleration());

//        double max = Math.max(1.0, Math.max(Math.abs(fLeftPower),
//                Math.max(Math.abs(bLeftPower), Math.max(Math.abs(fRightPower), Math.abs(bRightPower)))));
//
//        fLeftPower /= max;
//        bLeftPower /= max;
//        fRightPower /= max;
//        bRightPower /= max;

        Log.d("ppDebug", "fLeftPower: " + fLeftPower);

//        double basePowerMin = 0.4;
//        double distToGoal = Vector.between(currentPos, target).getLength();
//
//// taper the minimum from 40% (far away) down to 10% (once you’re <200 mm out)
//        double minPower = distToGoal > 200
//                ? basePowerMin
//                : 0.05 + 0.3 * (distToGoal / 200.0);

        driveTrain.setPowerWithRangeClippingMinThreshold(fLeftPower, fRightPower, bLeftPower, bRightPower, 0.25);

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


            int lastIdx = path.numPoints() - 1;
            int closestIdx = findClosestPointIndex(path, currentPosition);

            double elapsedTime = System.currentTimeMillis() - startTimeMS;

            if (elapsedTime >= maxTimeOutMS) {
                setIsDone(true);
                //Log.d("purepursaction_debug_follow", "done timeout  " + getName());
                return;
            }

            double nowMs      = elapsedTime;       // still in ms
            double dtSeconds  = (nowMs - lastMilli) / 1000.0;     // convert ms → s

            // guard against a zero‐division on the very first loop:
            if (dtSeconds <= 0) dtSeconds = 1e-3;  // 1 ms

            // compute distance travelled since last loop (in mm)
            Vector lastToCurrent = Vector.between(lastPosition, currentPosition);
            double distanceMm       = lastToCurrent.getLength();

            // now velocity in mm/s
            currentVelocityMmPerS = distanceMm / dtSeconds;

            currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM;

//            // inside update(), before lookAhead():
//            Vector toLast   = Vector.between(currentPosition, path.getLastPoint());
//            double segmentDir = toLast.getHeadingDirection();
//            double heading    = currentPosition.getTheta();
//
//            // angle between "forward" and your path direction:
//            double delta = Math.abs(MathFunctions.angleWrapRad(segmentDir - heading));

            // if you’re more than ±60° off forward, you’re really strafing:
//            if (Math.abs(Math.PI/2 - delta) < Math.toRadians(30)) {
//                currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM * 0.3;  // shrink to 30%
//            } else {
//                currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM;
//            }

            if (prevFollow.isPresent() && (path.findIndex(prevFollow.get()) > (path.numPoints() - 2))) {
                currentLookAheadRadius = lastSearchRadius;
            }

            follow = path.lookAhead(currentPosition, prevFollow, currentLookAheadRadius);

            if (follow.isPresent()) {
                targetPosition(follow.get(), currentPosition);
//            } else {
//                if (Math.abs(lastPoint.getTheta() - currentPosition.getTheta()) <= Math.toRadians(finalAngleLockingThreshholdDeg) ) {
//                    finishedMoving();
//                } else {
//                    targetPosition(lastPoint, currentPosition);
//                }
//            }
            } else if (closestIdx >= lastIdx) {
//                // we really are at the end of the path: switch to angle lock
//                double angleError = MathFunctions.angleWrapRad(
//                        path.getLastPoint().getTheta() - currentPosition.getTheta()
//                );
//                if (Math.abs(angleError) <= Math.toRadians(finalAngleLockingThreshholdDeg)) {
//                    finishedMoving();        // we’re within 1.5° of the final heading
//                } else {
//                    targetPosition(path.getLastPoint(), currentPosition);
//                }

                double now = elapsedTime;    // or System.currentTimeMillis()/1000.0
                double dt  = now - lastUpdateTime;
                lastUpdateTime = now;

                // only _now_ do your final heading‑lock
                double targetH = path.getLastPoint().getTheta();
                double err     = MathFunctions.angleWrapRad(targetH - currentPosition.getTheta());

                // special‑case EXACT ±π so sign doesn’t flip:
                if (Math.abs(Math.abs(err) - Math.PI) < 1e-3) {
                    err =  Math.PI;  // pick +π consistently
                }

                if (Math.abs(err) < Math.toRadians(finalAngleLockingThreshholdDeg)) {
                    Log.d("ppDebug", "finish by angle lock");
                    finishedMoving();
                } else {
                    // simple P‐turn: positive error → turn left, negative → turn right
                    double kP = 0.3, kD = 0.1;
                    double derivative = (err - lastHeadingError) / dt;
                    double turn = kP * err + kD * derivative;

// clamp and send
                    turn = Math.max(-0.5, Math.min(0.5, turn));
                    driveTrain.setPower(+turn, -turn, +turn, -turn);

                    lastHeadingError = err;
                }
            } else {
                // lost lookahead mid‑path (e.g. big deviation) – keep chasing the last follow point
                targetPosition(prevFollow.orElse(path.getLastPoint()), currentPosition);
            }


            xVelocity = (Math.abs(lastPosition.getX() - currentPosition.getX())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
            yVelocity = (Math.abs(lastPosition.getY() - currentPosition.getY())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
            thetaVelocity = (Math.abs(lastPosition.getTheta() - currentPosition.getTheta())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));


            if(xVelocity < 0.001 && yVelocity < 0.001 && thetaVelocity < 0.001) {
                if(timeoutTimer.milliseconds() > 1000) {
                    Log.d("ppDebug", "finish by velocity");
                    finishedMoving();
                }
            } else {
                timeoutTimer.reset();
            }

            lastMilli = nowMs;
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
            injectedPathPoints.add(SharedData.getOdometryPosition());
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
                path.getPoint(0).setAcceleration(0);

                if (calcPoint <= path.numPoints()-2) {
                    double d = path.getPoint(calcPoint+1).getDistanceAlongPath()
                            - path.getPoint(calcPoint).getDistanceAlongPath();
                    double vNext = path.getPoint(calcPoint+1).getVelocity();
                    // v² = vNext² + 2·a·d  →  v = sqrt(…)
                    double vMaxDecel = Math.sqrt(vNext*vNext + 2*MAX_ACCELERATION*d);

                    // after you compute vMaxDecel:
                    double signedV    = path.getPoint(calcPoint).getVelocity();
                    double sgn        = Math.signum(signedV);
                    double mag        = Math.abs(signedV);
                    double magLimited = Math.min(mag, vMaxDecel);

                    path.getPoint(calcPoint).setVelocity(magLimited);
                }

                double vPrev = path.getPoint(calcPoint-1).getVelocity();        // mm/s
                double sPrev = path.getPoint(calcPoint-1).getDistanceAlongPath(); // mm
                double vCurr = path.getPoint(calcPoint).getVelocity();          // mm/s
                double sCurr = path.getPoint(calcPoint).getDistanceAlongPath(); // mm

                double deltaS = sCurr - sPrev;                          // mm
                // avoid divide‑by‑zero for back‑to‑back identical points
                double accelMmPerS2 = (vCurr*vCurr - vPrev*vPrev) / (2.0 * deltaS);
                path.getPoint(calcPoint).setAcceleration(accelMmPerS2);
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
        if (positionIndex <= 0) {
            return 0;                // don’t drive “backwards” into start
        } else if (positionIndex == path.numPoints()-1) {
            return pathMaxVelocity;  // *do* drive at full speed into your goal
        }

        double curvature = Math.abs(curvature(path, positionIndex));
        if (curvature < 1e-6) return pathMaxVelocity; // straight line

        double mag = (positionIndex == path.numPoints()-1)
                ? pathMaxVelocity
                : Math.min(pathMaxVelocity, K / Math.abs(curvature(path, positionIndex)));

        // 2) compute the direction of that segment in *field* coordinates
        Position prev = path.getPoint(positionIndex-1);
        Position curr = path.getPoint(positionIndex);
        Vector seg    = Vector.between(prev, curr);

        // 3) project that onto the field-X axis (1,0).  If it’s negative, we need to go backwards:
        double sign = Math.signum(seg.getX());   // X‐axis dot seg == seg.getX()*1 + seg.getY()*0

        return mag;
    }

    private double getTargetVelocity(Path path, int positionIndex) {
        double v_i = path.getPoint(positionIndex-1).getVelocity();
        double a = MAX_ACCELERATION;
        double d = path.getPoint(positionIndex).getDistanceAlongPath() - path.getPoint(positionIndex-1).getDistanceAlongPath();

        double v_f = Math.sqrt(MathFunctions.square(v_i) + (2 * a * d));

        return Math.min(v_f, calculateVelocity(path, positionIndex));
    }

    private int findClosestPointIndex(Path path, Position current) {
        List<Position> pts = path.getPath();
        Position best = Collections.min(
                pts,
                Comparator.comparingDouble(p -> {
                    double dx = current.getX() - p.getX();
                    double dy = current.getY() - p.getY();
                    return dx*dx + dy*dy;
                })
        );
        return pts.indexOf(best);
    }

    private double calculateMotorOutput(double wheelVelocity, double acceleration) {
        return (K_p * (wheelVelocity - currentVelocityMmPerS) + K_v * wheelVelocity + K_a * Math.signum(wheelVelocity) * Math.abs(acceleration));
    }
}