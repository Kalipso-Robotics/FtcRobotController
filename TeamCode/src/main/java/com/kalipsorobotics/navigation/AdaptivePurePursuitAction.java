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
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

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

    private boolean enteredFinalAngleLock = false;

    private double pathMaxVelocity = 2000;
    // If the robot overshoots or skids in curves → lower it, if the robot is slow or choppy in straightaways → raise it
    private final double K = 1100; //based on how slow you want the robot to go around turns
    // If robot cuts corners or skids → reduce K, if robot slows down too much in gentle curves → increase K
    private static final double MAX_ACCELERATION = 10000; // mm/s^2
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

    private int calcDistanceIndex = 0;
    private boolean calcDistanceDone = false;

    private int calcVAIndex = -1;
    private boolean calcVelocityAccelDone = false;
    private double lastUpdateTime;
    private double lastHeadingError = 0;

    private double WHEELBASE_LENGTH = 300; //front wheel to back wheel
    private double TRACK_WIDTH = 400; //side to side
    private double K_p = 0; // 0.000015
    private double K_a = 0.0; // 0.001
    private double K_v = 0.00225; // 0.0004

    /*
    * ↑ Raising K_p
    * The controller reacts more aggressively to any mismatch between your commanded wheelVelocity and what you’re actually seeing.
    * Pros: tighter tracking, smaller steady‐state velocity error.
    * Cons: if too high, you’ll get oscillations or a chattery response as you constantly “hunting” your target speed.

    * ↓ Lowering K_p
    * Makes that feedback loop more sluggish.
    * Pros: smoother, less jitter.
    * Cons: larger lag and steady‐state error (you’ll always run a bit slower than commanded).
    */

    /*
    * ↑ Raising K_v
    * Increases the baseline motor power you send for any given wheelVelocity.
    * Pros: you’ll hit your commanded speed more easily, even on hills or friction.
    * Cons: if too high, your outputs will clip at ±1.0 and corners of your profile will get “flattened” or you’ll overshoot.

    * ↓ Lowering K_v
    * Hands more of the work over to the feedback term (K_p).
    * Pros: less risk of raw saturation, more graceful cornering of your velocity trapezoid.
    * Cons: if too low, you’ll visibly struggle to reach or hold top speed, even when unloaded.
    */

    /*
    * ↑ Raising K_a
    * Kicks in extra power proportionally to how sharply you’re accelerating or braking along the path.
    * Pros: helps overcome inertia and get those velocity ramps on‐profile.
    * Cons: if too high, you’ll see spikes in power whenever your path curvature or speed changes, which can feel jerky.

    * ↓ Lowering K_a
    * Makes your accelerations more reliant on the feedback loop to “catch up.”
    * Pros: smoother, avoids power‐spikes.
    * Cons: your robot may feel sluggish at the beginning of each motion segment or in tight decelerations.
     */

    /*
    If you see a lot of oscillation around your commanded speed, back K_p down until it damps out.
    If you can’t ever quite reach the velocity plateau, raise K_v a bit.
    If your robot hesitates or “drifts” through ramps, boost K_a to help it follow your acceleration profile.
     */

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

        Log.d("ppDebug", "angleError: " + angleError);

        double omega = 0;
        if (Math.abs(angleError) > Math.toRadians(5)) {  // only correct if >5°
            omega = target.getPidAngle().getPower(angleError);
            Log.d("ppDebug", "omega set due to angle error: " + omega);
        }

        Log.d("ppDebug", "vx: " + vx);
        Log.d("ppDebug", "vy: " + vy);
        Log.d("ppDebug", "omega: " + omega);



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

        Log.d("ppDebug", "velocity target: " + target.getVelocity());
        Log.d("ppDebug", "velocity current: " + currentVelocityMmPerS);

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

        if (injectDone && smootherDone && !calcDistanceDone) {
            calculateDistanceAlongPath(path);
            return;
        }

        if (injectDone && smootherDone && calcDistanceDone && !calcVelocityAccelDone) {
            calculateVelocityAcceleration(path);
            return;
        }

        if (injectDone && smootherDone && calcDistanceDone && calcVelocityAccelDone) {
            currentPosition = new Position(SharedData.getOdometryPosition());


            int lastIdx = path.numPoints() - 1;
            int closestIdx = findClosestPointIndex(path, currentPosition);

            double elapsedTime = System.currentTimeMillis() - startTimeMS;

            if (elapsedTime >= maxTimeOutMS) {
                setIsDone(true);
                //Log.d("purepursaction_debug_follow", "done timeout  " + getName());
                return;
            }

            // still in ms
            double dtSeconds  = (elapsedTime - lastMilli) / 1000.0;     // convert ms → s

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

            if (!enteredFinalAngleLock) {

                if (follow.isPresent()) {

                    Log.d("ppDebug", "follow: " + path.findIndex(follow.get()) + ": " + follow.get().getPoint());

                    if (follow.get() == path.getLastPoint() && currentPosition.distanceTo(follow.get()) > LAST_RADIUS_MM) {
                        targetPosition(path.getPoint(path.findIndex(path.getLastPoint()) - 1), currentPosition);  // skip the zero-speed goal until you’re nearby
                        Log.d("ppDebug", "follow second last point");

                    } else {

                        targetPosition(follow.get(), currentPosition);
                        Log.d("ppDebug", "follow found point");

                    }

                } else if (closestIdx >= lastIdx) {
                    enteredFinalAngleLock = true;
                } else {
                    // lost lookahead mid‑path (e.g. big deviation) – keep chasing the last follow point
                    targetPosition(prevFollow.orElse(path.getLastPoint()), currentPosition);
                    Log.d("ppDebug", "follow not found -> prev point");
                }
            } else {
                ////                // we really are at the end of the path: switch to angle lock
//                double angleError = MathFunctions.angleWrapRad(
//                        path.getLastPoint().getTheta() - currentPosition.getTheta()
//                );
//                if (Math.abs(angleError) <= Math.toRadians(finalAngleLockingThreshholdDeg)) {
//                    Log.d("ppDebug", "finish by angle lock");
//                    finishedMoving();        // we’re within 1.5° of the final heading
//                } else {
//                    targetPosition(path.getLastPoint(), currentPosition);
//                }
//
//                // or System.currentTimeMillis()/1000.0
//                double dt = elapsedTime - lastUpdateTime;
//                lastUpdateTime = elapsedTime;

                // only _now_ do your final heading‑lock
                double targetH = path.getLastPoint().getTheta();
                double err = MathFunctions.angleWrapRad(targetH - currentPosition.getTheta());
                Log.d("ppDebug", "follow not found -> last point, angle error: " + err);

                // special‑case EXACT ±π so sign doesn’t flip:
                if (Math.abs(Math.abs(err) - Math.PI) < 1e-3) {
                    err = Math.PI;  // pick +π consistently
                }

                if (Math.abs(err) < Math.toRadians(finalAngleLockingThreshholdDeg)) {
                    Log.d("ppDebug", "finish by angle lock");
                    finishedMoving();
                } else {
//                    // simple P‐turn: positive error → turn left, negative → turn right
//                    double kP = 0.3, kD = 0.1;
//                    double derivative = (err - lastHeadingError) / dt;
//                    double turn = kP * err + kD * derivative;
//
//                    // clamp and send
//                    turn = Range.clip(turn, -0.5, 0.5);
                    double turn = path.getLastPoint().getPidAngle().getPower(err);
//                    driveTrain.setPower(+turn, -turn, +turn, -turn);
                    driveTrain.setPowerWithRangeClippingMinThreshold(turn, -turn, turn, -turn, 0.15);

                    //lastHeadingError = err;
                }
            }

            xVelocity = (Math.abs(lastPosition.getX() - currentPosition.getX())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
            yVelocity = (Math.abs(lastPosition.getY() - currentPosition.getY())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
            thetaVelocity = (Math.abs(lastPosition.getTheta() - currentPosition.getTheta())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));

//            if(xVelocity < 0.001 && yVelocity < 0.001 && thetaVelocity < 0.001) {
//                if(timeoutTimer.milliseconds() > 5000) {
//                    Log.d("ppDebug", "finish by velocity");
//                    finishedMoving();
//                }
//            } else {
//                timeoutTimer.reset();
//            }

            lastMilli = elapsedTime;
            lastPosition = currentPosition;
        }

        Log.d("adaptive pure pursuit", "inject done: " + injectDone);
        Log.d("adaptive pure pursuit", "smoother done: " + smootherDone);
        Log.d("adaptive pure pursuit", "velocity done: " + calcVelocityAccelDone);

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
        int spacingMM = 200;

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
                Log.d("ppDebug", "injected point: " + x + ", " + y + ", " + theta);

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

    private void calculateDistanceAlongPath(Path path) {
        if (calcDistanceIndex <= path.numPoints()-1) {
            if (calcDistanceIndex == 0) {
                path.getPoint(calcDistanceIndex).setDistanceAlongPath(0);
            } else {
                Vector vector = Vector.between(path.getPoint(calcDistanceIndex - 1), path.getPoint(calcDistanceIndex));
                path.getPoint(calcDistanceIndex).setDistanceAlongPath(path.getPoint(calcDistanceIndex - 1).getDistanceAlongPath() + vector.getLength());

                Log.d("ppDebug", "set distance of point " + calcDistanceIndex + " to: " + (path.getPoint(calcDistanceIndex - 1).getDistanceAlongPath() + vector.getLength()));
            }

            calcDistanceIndex++;
        } else {
            calcDistanceDone = true;
        }
    }

    private void calculateVelocityAcceleration(Path path) {
        if (calcVAIndex < 0 && !calcVelocityAccelDone) {
            calcVAIndex = path.numPoints()-1;
            Log.d("adaptive pure pursuit velocity", "set calcpoint to last point: " + calcVAIndex);
        }

        if (calcVAIndex >=0) {
            if (calcVAIndex == 0) {
                path.getPoint(calcVAIndex).setCurvature(curvature(path, calcVAIndex));
                path.getPoint(calcVAIndex).setVelocity(0);

                calcVelocityAccelDone = true;
            } else {
                path.getPoint(calcVAIndex).setCurvature(curvature(path, calcVAIndex));

                path.getPoint(calcVAIndex).setVelocity(getTargetVelocity(path, calcVAIndex));
                path.getPoint(0).setAcceleration(0);

                if (calcVAIndex <= path.numPoints()-2) {
                    double d = path.getPoint(calcVAIndex +1).getDistanceAlongPath()
                            - path.getPoint(calcVAIndex).getDistanceAlongPath();
                    double vNext = path.getPoint(calcVAIndex +1).getVelocity();
                    // v² = vNext² + 2·a·d  →  v = sqrt(…)
                    double vMaxDecel = Math.sqrt(vNext*vNext + 2*MAX_ACCELERATION*d);

                    // after you compute vMaxDecel:
                    double signedV    = path.getPoint(calcVAIndex).getVelocity();
                    double sgn        = Math.signum(signedV);
                    double mag        = Math.abs(signedV);
                    double magLimited = Math.min(mag, vMaxDecel);

                    path.getPoint(calcVAIndex).setVelocity(magLimited);

                }

                double vPrev = path.getPoint(calcVAIndex -1).getVelocity();        // mm/s
                double sPrev = path.getPoint(calcVAIndex -1).getDistanceAlongPath(); // mm
                double vCurr = path.getPoint(calcVAIndex).getVelocity();          // mm/s
                double sCurr = path.getPoint(calcVAIndex).getDistanceAlongPath(); // mm

                Log.d("ppDebug", "set acceleration of point " + calcVAIndex + " using vPrev: " + vPrev + ", sPrev: " + sPrev+ ", vCurr: " + vCurr+ ", sCurr: " + sCurr);

                double deltaS = sCurr - sPrev;                          // mm
                // avoid divide‑by‑zero for back‑to‑back identical points
                double accelMmPerS2 = (vCurr*vCurr - vPrev*vPrev) / (2.0 * deltaS);
                path.getPoint(calcVAIndex).setAcceleration(accelMmPerS2);
                Log.d("ppDebug", "set acceleration of point " + calcVAIndex + " to: " + accelMmPerS2);

            }

            Log.d("adaptive pure pursuit velocity", "calculated point: " + calcVAIndex);
            calcVAIndex--;

        } else {
            calcVelocityAccelDone = true;
        }
    }

    public static double curvature(Path path, int positionIndex) {

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
            return 0;  // *do* drive at full speed into your goal
        }

        double curvature = Math.abs(curvature(path, positionIndex));
        if (curvature < 1e-6) return pathMaxVelocity; // straight line

        double mag = Math.min(pathMaxVelocity, K / Math.abs(curvature(path, positionIndex)));

        return mag;
    }

    private double getTargetVelocity(Path path, int positionIndex) {
        double v_i = path.getPoint(positionIndex-1).getVelocity();
        double d = path.getPoint(positionIndex).getDistanceAlongPath() - path.getPoint(positionIndex-1).getDistanceAlongPath();

        double v_f = Math.sqrt(MathFunctions.square(v_i) + (2 * MAX_ACCELERATION * d));

        return Math.min(v_f, calculateVelocity(path, positionIndex));
    }

    private int findClosestPointIndex(Path path, Position current) {
        List<Position> pts = path.getPath();

        return IntStream.range(0, pts.size())
                .boxed()
                .min(
                        Comparator
                                // 1) compare by distance²
                                .comparingDouble((Integer i) -> {
                                    Position p = pts.get(i);
                                    double dx = current.getX() - p.getX();
                                    double dy = current.getY() - p.getY();
                                    return dx*dx + dy*dy;
                                })
                                // 2) if distances tie, prefer the higher index
                                .thenComparing(Comparator.reverseOrder())
                )
                .orElse(0);
    }

    private double calculateMotorOutput(double wheelVelocity, double acceleration) {
        return (K_p * (wheelVelocity - currentVelocityMmPerS) + K_v * wheelVelocity + K_a * Math.signum(wheelVelocity) * Math.abs(acceleration));
    }
}