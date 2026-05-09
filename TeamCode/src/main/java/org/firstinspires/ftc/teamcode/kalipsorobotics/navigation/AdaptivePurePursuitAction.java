package org.firstinspires.ftc.teamcode.kalipsorobotics.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Path;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

public class AdaptivePurePursuitAction extends Action {

    DriveTrain driveTrain;
    Odometry wheelOdometry;

    List<Position> pathPoints = new ArrayList<Position>();

    Path path;
    static final private double LOOK_AHEAD_RADIUS_MM = 150;

    private double currentLookAheadRadius;
    static final private double LAST_RADIUS_MM = 15;

    private Position currentPosition = new Position(SharedData.getOdometryWheelIMUPosition());

    Optional<Position> follow;
    Optional<Position> prevFollow;

    private final double lastSearchRadius = LAST_RADIUS_MM;

    private final double finalAngleLockingThreshholdDeg = 1.5;

    private double startTimeMS = System.currentTimeMillis();
    private final double maxTimeOutMS = 1000000000;

    private Position lastPosition;
    private double lastMilli = 0;
    ElapsedTime timeoutTimer;

    private double currentVelocityMmPerS;
    private double filteredVelocityMmPerS = 0;
    private final double FILTER_SMOOTHING_FACTOR = 0.8; // A value between 0.0 and 1.0
    private static final double VELOCITY_DEADBAND = 5.0;

    List<Position> injectedPathPoints = new ArrayList<>();
    private int pointInject = 0;
    private double injectDistance = 0;
    private int segInject = 0;
    private boolean injectDone = false;

    Path newPath = null;
    private boolean finishedCurrentLoop = false;
    private final double SMOOTHER_A = 0.25;
    private final double SMOOTHER_B = 1 - SMOOTHER_A;
    private final double SMOOTHER_CORNER_PULL = 5.0;
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
    private final double lastHeadingError = 0;


    //TUNING NUMBERS: USE DATA ABOUT ROBOT
    private final double PATH_MAX_VELOCITY = 1500; // If the robot overshoots or skids in curves → lower it, if the robot is slow or choppy in straightaways → raise it
    // If robot cuts corners or skids → reduce K, if robot slows down too much in gentle curves → increase K
    // 2000
    private final double MAX_ACCELERATION = 6000; // mm/s^2, maximum acceleration of the robot, 7500
    private final double MAX_ACCELERATION_FINAL = MAX_ACCELERATION / 3; // mm/s^2
    // If the robot struggles to accelerate → lower a, if it's too conservative and slow → raise a
    private final double MAX_ANGULAR_VELOCITY = 5.5; //rad/s, maximum turning velocity of the robot

    private final double WHEELBASE_LENGTH = 300; //front wheel to back wheel
    private final double TRACK_WIDTH = 400; //side to side
    private final double K_p = 0.00002; // 0.000015
    private final double K_a = 0.0; // 0.001
    private final double K_v = 0.0004; // 0.0004 0.00225
    private final double K = 1000; //based on how slow you want the robot to go around turns, 1100

    /*
    * ↑ Raising K_p
    * Pros: tighter tracking, smaller steady‐state velocity error.
    * Cons: if too high, oscillations or a chattery response as bot constantly “hunts” target speed.
    * ↓ Lowering K_p
    * Pros: smoother, less jitter.
    * Cons: larger lag and steady‐state error (always run a bit slower than commanded).
    */

    /*
    * ↑ Raising K_v
    * Pros: hit your commanded speed more easily, even on hills or friction.
    * Cons: if too high, outputs will clip at ±1.0 and corners of profile will get “flattened” or overshoot.
    * ↓ Lowering K_v
    * Pros: less risk of raw saturation, more graceful cornering of your velocity trapezoid.
    * Cons: if too low, visibly struggle to reach or hold top speed, even when unloaded.
    */

    /*
    * ↑ Raising K_a
    * Pros: helps overcome inertia and get those velocity ramps on‐profile.
    * Cons: if too high, spikes in power when path curvature or speed changes, which can feel jerky.
    * ↓ Lowering K_a
    * Pros: smoother, avoids power‐spikes.
    * Cons: may feel sluggish at the beginning of each motion segment or in tight decelerations.
     */

    public AdaptivePurePursuitAction(DriveTrain driveTrain, Odometry wheelOdometry) {
        this.driveTrain = driveTrain;
        this.wheelOdometry = wheelOdometry;

        this.timeoutTimer = new ElapsedTime();
        lastUpdateTime = timeoutTimer.milliseconds();

        this.follow = Optional.empty();
        this.prevFollow = Optional.empty();

        this.dependentActions.add(new DoneStateAction());
    }

    public void addPoint(double x, double y, double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        pathPoints.add(new Position(x, y, headingRad));
    }

    @Override
    public void setIsDone(boolean isDone) {
        this.isDone = isDone;
        if (isDone) {
            driveTrain.setPower(0);
        }
    }

    public void reset() {
        // Reset all boolean flags
        isDone = false;
        hasStarted = false;
        injectDone = false;
        smootherDone = false;
        calcDistanceDone = false;
        calcVelocityAccelDone = false;

        // Clear and reset data structures
        injectedPathPoints.clear();
        path = null;
        newPath = null;
        follow = Optional.empty();
        prevFollow = Optional.empty();

        // Reset counters and temporary values
        pointInject = 0;
        injectDistance = 0;
        segInject = 0;
        finishedCurrentLoop = false;
        change = SMOOTHER_TOLERANCE;
        smootherI = 1;
        smootherJ = 0;
        calcDistanceIndex = 0;
        calcVAIndex = -1;
        lastMilli = 0;
        lastPosition = null; // or new Position(...)

        // Reset timers
        timeoutTimer.reset();
        lastUpdateTime = timeoutTimer.milliseconds();
    }

    private void targetPosition(Position target, Position currentPos) {

        double velocity;

        if (target == path.getLastPoint()) { //if the current target is the very last point

            // Final approach: Use a kinematic model for deceleration to a stop
            double distanceToEnd = Vector.between(currentPos, path.getLastPoint()).getLength();

            // Calculate the maximum velocity allowed to decelerate and stop at the target
            velocity = Math.sqrt(2 * MAX_ACCELERATION_FINAL * distanceToEnd);

        } else {
            velocity = target.getVelocity();
        }

        double robotAngle = currentPos.getTheta();
        KLog.d("ppDebug", () -> "robotAngle: " + robotAngle);

        // Robot‑relative vector to lookahead (in meters)
        double dx = target.getX() - currentPos.getX();
        double dy = target.getY() - currentPos.getY();

        // Rotate into robot frame
        double x_r =  Math.cos(robotAngle)*dx + Math.sin(robotAngle)*dy;
//        Log.d("ppDebug", "x_r: " + x_r);
        double y_r = -Math.sin(robotAngle)*dx + Math.cos(robotAngle)*dy;
//        Log.d("ppDebug", "y_r: " + y_r);

        double headingError = Math.atan2(y_r, x_r);
        // Choose speeds
//        double cosHE = Math.cos(headingError);
//        Log.d("ppDebug", "cosHE: " + cosHE);
//
//        double vx = velocity * cosHE;
//        double vy = velocity * Math.sin(headingError);

        // The vector to the lookahead point in the robot's local frame
        // This vector has components (x_r, y_r).
        double lookaheadDist = Math.sqrt(x_r * x_r + y_r * y_r);

        // If we're far from the target, scale the velocities based on the lookahead vector.
        // This is the true Pure Pursuit approach for linear velocity.
        double vx;
        double vy;
        if (lookaheadDist > 1e-6) {
            vx = velocity * (x_r / lookaheadDist);
            vy = velocity * (y_r / lookaheadDist);
        } else {
            vy = 0;
            vx = 0;
        }

        double angleError = MathFunctions.angleWrapRad(
                target.getTheta() - currentPos.getTheta()
        );

//        Log.d("ppDebug", "angleError: " + angleError);
//        Log.d("ppDebug", "headingError: " + headingError);

        // Calculate the desired angular velocity based on the PID output
        double omegaAngularVelocity = target.getPidAngleAdaptive().getPower(angleError) * MAX_ANGULAR_VELOCITY; // This is in rad/s

        // Convert the desired angular velocity (rad/s) into a linear velocity (mm/s)
        double K_omega = Math.sqrt(MathFunctions.square(WHEELBASE_LENGTH) + MathFunctions.square(TRACK_WIDTH)) / 2; // This value is in mm/rad

        double omega = omegaAngularVelocity * K_omega; // This is now in mm/s

        KLog.d("ppDebug", () -> "vx: " + vx);
        KLog.d("ppDebug", () -> "vy: " + vy);
        KLog.d("ppDebug", () -> "omega: " + omega);

//        double fLeftVelocity = vx + vy + ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;
//        double bLeftVelocity = vx - vy + ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;
//        double fRightVelocity = vx - vy - ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;
//        double bRightVelocity = vx + vy - ((WHEELBASE_LENGTH + TRACK_WIDTH) / 2) * omega;

        double fLeftVelocity = vx + vy + omega;
        double bLeftVelocity = vx - vy + omega;
        double fRightVelocity = vx - vy - omega;
        double bRightVelocity = vx + vy - omega;

        KLog.d("wheels", () -> "fLeftVelocity: " + fLeftVelocity);
        KLog.d("wheels", () -> "bLeftVelocity: " + bLeftVelocity);
        KLog.d("wheels", () -> "fRightVelocity: " + fRightVelocity);
        KLog.d("wheels", () -> "bRightVelocity: " + bRightVelocity);

        KLog.d("ppDebug", () -> "acceleration: " + target.getAcceleration());

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

        KLog.d("wheels", () -> String.format("Motor powers: FL=%.3f, FR=%.3f, BL=%.3f, BR=%.3f",
            fLeftPower, fRightPower, bLeftPower, bRightPower));

        driveTrain.setPowerWithRangeClippingMinThreshold(fLeftPower, fRightPower, bLeftPower, bRightPower, 0.25);

        KLog.d("ppDebug", () -> String.format("Target vel=%.1f mm/s, Current vel=%.1f mm/s, Accel=%.1f mm/s²",
            velocity, filteredVelocityMmPerS, target.getAcceleration()));

        prevFollow = Optional.of(target);
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            reset();
            startTimeMS = System.currentTimeMillis();
            hasStarted = true;
            lastPosition = new Position(SharedData.getOdometryWheelIMUPosition());
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
            currentPosition = new Position(SharedData.getOdometryWheelIMUPosition());
            KLog.d("ppDebug", () -> "currentPosition: " + currentPosition);

            int lastIdx = path.numPoints() - 1;
            int closestIdx = findClosestPointIndex(path, currentPosition);
            KLog.d("ppDebug", () -> "closestIdx index: " + closestIdx);

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

            if (distanceMm < 0.1) {
                double finalDtSeconds = dtSeconds;
                KLog.d("ppDebugStuck", () -> String.format("WARNING: Robot not moving! Distance=%.3fmm in %.3fs",
                    distanceMm, finalDtSeconds));
            }

            if (Math.abs(currentVelocityMmPerS) > VELOCITY_DEADBAND) {
                filteredVelocityMmPerS = (FILTER_SMOOTHING_FACTOR * filteredVelocityMmPerS) +
                        ((1 - FILTER_SMOOTHING_FACTOR) * currentVelocityMmPerS);
            }

            currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM;

            follow = path.lookAhead(currentPosition, prevFollow, currentLookAheadRadius);

            KLog.d("ppDebugFollow", () -> String.format("Lookahead: radius=%.1f, found=%s, prevFollow=%s",
                currentLookAheadRadius,
                follow.isPresent() ? "YES" : "NO",
                prevFollow.isPresent() ? prevFollow.get().getPoint() : "NONE"));

            // Check for the final point
            // If lookAhead finds a point AND that point is the last point,
            // it means we should shrink our search radius for the next iteration.
            // This is a more robust way to handle the end of the path.
            if (follow.isPresent() && follow.get() == path.getLastPoint()) {
                KLog.d("ppDebugFollow", "Follow point is last point, shrinking radius");
                currentLookAheadRadius = LAST_RADIUS_MM;
                follow = path.lookAhead(currentPosition, prevFollow, currentLookAheadRadius);
                KLog.d("ppDebugFollow", () -> String.format("After shrink: radius=%.1f, found=%s",
                    currentLookAheadRadius, follow.isPresent() ? "YES" : "NO"));
            }

            if (follow.isPresent()) {
                int followIndex = path.getIndex(follow.get());
                int lastIndex = path.numPoints() - 1;

                KLog.d("ppDebugFollow", () -> String.format("Following point %d/%d: (%.1f, %.1f, %.1f°)",
                    followIndex, lastIndex,
                    follow.get().getX(), follow.get().getY(), Math.toDegrees(follow.get().getTheta())));

                // We have a point to follow
                targetPosition(follow.get(), currentPosition);

            } else {
                // If no lookahead point is found, we are at the end of the path.
                // Command the robot to drive to the very last point.
                double angleError = MathFunctions.angleWrapRad(
                        path.getLastPoint().getTheta() - currentPosition.getTheta()
                );

                Vector between = Vector.between(currentPosition, path.getLastPoint());
                double distanceToEnd = between.getLength();

                KLog.d("ppDebugFollow", () -> String.format("NO LOOKAHEAD! Final lock: angleErr=%.1f° (thresh=%.1f°), dist=%.1fmm (thresh=%.1fmm)",
                    Math.toDegrees(angleError), finalAngleLockingThreshholdDeg,
                    distanceToEnd, lastSearchRadius));

                if (Math.abs(angleError) <= Math.toRadians(finalAngleLockingThreshholdDeg) && distanceToEnd < lastSearchRadius) {
                    KLog.d("ppDebugFollow", "FINISHING MOVEMENT - within thresholds");
                    finishedMoving();
                } else {
                    KLog.d("ppDebugFollow", "Driving to final point");
                    targetPosition(path.getLastPoint(), currentPosition);
                }
            }


//            xVelocity = (Math.abs(lastPosition.getX() - currentPosition.getX())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
//            yVelocity = (Math.abs(lastPosition.getY() - currentPosition.getY())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
//            thetaVelocity = (Math.abs(lastPosition.getTheta() - currentPosition.getTheta())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));

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

    }

    public void finishedMoving() {
        driveTrain.setPower(0);
        isDone = true;
    }

    public Optional<Position> getCurrentFollowPoint() {
        return follow;
    }

    public Path getPath() {
        return path;
    }

    public Optional<Position> getClosestPathPoint() {
        if (path == null || !calcVelocityAccelDone) {
            return Optional.empty();
        }
        Position current = new Position(SharedData.getOdometryWheelIMUPosition());
        int closestIdx = findClosestPointIndex(path, current);
        return Optional.of(path.getPoint(closestIdx));
    }


    private void injectPoints(Path path) {
//        int spacingMM = 100;

        if (segInject == 0 && injectDistance == 0 && injectedPathPoints.isEmpty()) {
            Position robotStart = new Position(SharedData.getOdometryWheelIMUPosition());
            injectedPathPoints.add(robotStart);

            // Add the very first point from the original path only once
            Position pathFirstPoint = path.getPoint(0);
            if (Vector.between(robotStart, pathFirstPoint).getLength() > 1e-6) {
                injectedPathPoints.add(pathFirstPoint);
            }

            pointInject = 1;
        }

        if (segInject < path.numSegments()) {
            Vector vector = path.getSegment(segInject).getVector();
            double segmentLength = vector.getLength();
//            double eps = 1e-6;
//            double numPointsFit = (int) Math.floor((segmentLength - eps) / spacingMM);
//            //Log.d("ppDebug", "num points fit: " + numPointsFit);
//
//            if (Math.abs(numPointsFit * spacingMM - segmentLength) < 1e-6) {
//                numPointsFit--;
//            }

            if (segmentLength < 1e-6) {
                Position end = path.getSegment(segInject).getFinish();
                injectedPathPoints.add(end);
                segInject++;
                injectDistance = 0;
                return;
            }

            Vector norm = vector.normalize();
//            Vector unitVector = new Vector(norm.getX() * spacingMM, norm.getY() * spacingMM);

            Position start = path.getSegment(segInject).getStart();
            Position end = path.getSegment(segInject).getFinish();

            double startTheta = start.getTheta();
            double endTheta = end.getTheta();

            double nextSpacing = getSpacingForDistance(injectDistance, segmentLength, segInject, path.numSegments());
            double nextDistance = injectDistance + nextSpacing;

//            int i = pointInject + ((segInject == 0) ? 1 : 0);

            if (nextDistance < segmentLength - 1e-6) {
                double x = start.getX() + norm.getX() * nextDistance;
                double y = start.getY() + norm.getY() * nextDistance;

//                double t = pointInject / numPointsFit;
//                double theta = MathFunctions.interpolateAngle(startTheta, endTheta, t);

                double thetaT = nextDistance / segmentLength;
                thetaT = Math.max(0.0, Math.min(1.0, thetaT));
                double theta = MathFunctions.interpolateAngle(startTheta, endTheta, thetaT);

//                double theta;
//                if (MAX_ANGULAR_VELOCITY * pointInject > endTheta) {
//                    theta = endTheta;
//                } else {
//                    theta = MAX_ANGULAR_VELOCITY * pointInject;
//                }

                injectedPathPoints.add(new Position(x, y, theta));
                KLog.d("ppDebug", () -> "injected point: " + x + ", " + y + ", " + theta);

                injectDistance = nextDistance;
            } else {
                injectedPathPoints.add(end);
                KLog.d("ppDebug", () -> "segment endpoint: " + end.getX() + ", " + end.getY() + ", " + end.getTheta());

                injectDistance = 0;
                segInject++;
            }

        } else {
            injectDone = true;

//            Position finalOriginalPoint = path.getLastPoint();
//            Position lastInjectedPoint = injectedPathPoints.get(injectedPathPoints.size() - 1);
//
//            // If the last injected point is NOT the same as the final original point,
//            // then we need to add the final original point.
//            final double EPSILON = 1e-6;
//            if (Math.abs(lastInjectedPoint.getX() - finalOriginalPoint.getX()) > EPSILON ||
//                    Math.abs(lastInjectedPoint.getY() - finalOriginalPoint.getY()) > EPSILON) {
//
//                injectedPathPoints.add(finalOriginalPoint);
//            }
//
//            injectedPathPoints.add(path.getLastPoint());
//
//            KLog.d("ppDebug", () -> "injected last point: " + path.getLastPoint().getX() + ", " + path.getLastPoint().getY() + ", " + path.getLastPoint().getTheta());

        }
    }

    private double getSpacingForDistance(double distanceAlongSegment, double segmentLength, int segInject, int totalSegments) {
        double smallSpacing = 50.0;
        double largeSpacing = 100.0;

        double distToEnd = segmentLength - distanceAlongSegment;

        if ((!(segInject == 0) && distanceAlongSegment < 2 * smallSpacing) || (!(segInject == totalSegments - 1) && distToEnd <= largeSpacing + 2 * smallSpacing)) {
            return smallSpacing;
        }


        return largeSpacing;
    }

    private void smoother(Path path) {

        if (newPath == null && injectDone) {
            newPath = new Path(path.getPath());
        }

        if (injectDone && (!finishedCurrentLoop || change >= SMOOTHER_TOLERANCE)) {

            if (finishedCurrentLoop && change >= SMOOTHER_TOLERANCE) {
                smootherI = 1; //skip 0 don't smooth first point
                change = 0.0;
                finishedCurrentLoop = false;
            }

            if (smootherI < path.numPoints()-1) { //skip last point

                // Add a check to skip original waypoints, small tolerance for floating-point comparison
                boolean isOriginal = false;
                for (Position original : pathPoints) {
                    if (Math.abs(path.getPoint(smootherI).getX() - original.getX()) < 1e-6 &&
                            Math.abs(path.getPoint(smootherI).getY() - original.getY()) < 1e-6) {
                        isOriginal = true;
                        break;
                    }
                }

                if (isOriginal) {
                    // Skip smoothing this point
                    smootherJ = 0;
                    smootherI++;
                    return;
                }

                Vector cornerBias = getCornerBias(path, smootherI);
                double cornerWeight = getCornerWeight(path, smootherI);
                double localA = getOriginalPullWeight(path, smootherI);
                double localB = getNeighborPullWeight(path, smootherI);

                if (smootherJ <= 1) {
                    if (smootherJ == 0) {
                        double aux = newPath.getPoint(smootherI).getX();
                        newPath.getPoint(smootherI).addX(localA * (path.getPoint(smootherI).getX() - newPath.getPoint(smootherI).getX())
                                + localB * (newPath.getPoint(smootherI-1).getX() + newPath.getPoint(smootherI+1).getX() - (2.0 * newPath.getPoint(smootherI).getX()))
                        + SMOOTHER_CORNER_PULL * cornerWeight * cornerBias.getX());
                        change += Math.abs(aux - newPath.getPoint(smootherI).getX());
                    } else {
                        double aux = newPath.getPoint(smootherI).getY();
                        newPath.getPoint(smootherI).addY(localA * (path.getPoint(smootherI).getY() - newPath.getPoint(smootherI).getY())
                                + localB * (newPath.getPoint(smootherI-1).getY() + newPath.getPoint(smootherI+1).getY() - (2.0 * newPath.getPoint(smootherI).getY()))
                        + SMOOTHER_CORNER_PULL * cornerWeight * cornerBias.getY());
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

    private double getOriginalPullWeight(Path path, int pointIndex) {
        if (pointIndex + 1 < path.numPoints()) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex + 1)) != -1) {
                return 0.025;
            }
        }
        if (pointIndex - 1 >= 0) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex - 1)) != -1) {
                return 0.025;
            }
        }

        if (pointIndex + 2 < path.numPoints()) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex + 2)) != -1) {
                return 0.05;
            }
        }
        if (pointIndex - 2 >= 0) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex - 2)) != -1) {
                return 0.05;
            }
        }

        return SMOOTHER_A;
    }

    private double getNeighborPullWeight(Path path, int pointIndex) {
        if (pointIndex + 1 < path.numPoints()) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex + 1)) != -1) {
                return 0.2;
            }
        }
        if (pointIndex - 1 >= 0) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex - 1)) != -1) {
                return 0.2;
            }
        }

        if (pointIndex + 2 < path.numPoints()) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex + 2)) != -1) {
                return 0.3;
            }
        }
        if (pointIndex - 2 >= 0) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex - 2)) != -1) {
                return 0.3;
            }
        }

        return SMOOTHER_B;
    }

    private int getOriginalWaypointIndex(Position p) {
        for (int i = 0; i < pathPoints.size(); i++) {
            Position original = pathPoints.get(i);
            if (Math.abs(p.getX() - original.getX()) < 1e-6 &&
                    Math.abs(p.getY() - original.getY()) < 1e-6) {
                return i;
            }
        }
        return -1;
    }

    private Vector getCornerBias(Path path, int pointIndex) {
        final double EPS = 1e-6;

        int originalIdx = -1;

        // find which waypoint the corner is
        if (pointIndex + 1 < path.numPoints()) {
            originalIdx = getOriginalWaypointIndex(path.getPoint(pointIndex + 1));
        }
        if (originalIdx == -1 && pointIndex - 1 >= 0) {
            originalIdx = getOriginalWaypointIndex(path.getPoint(pointIndex - 1));
        }

        // must be an actual corner, not start or end
        if (originalIdx <= 0 || originalIdx >= pathPoints.size() - 1) {
            return new Vector(0, 0);
        }

        Position fixedPrev = pathPoints.get(originalIdx - 1);
        Position fixed = pathPoints.get(originalIdx);
        Position fixedNext = pathPoints.get(originalIdx + 1);

        Vector in = Vector.between(fixedPrev, fixed).normalize();
        Vector out = Vector.between(fixed, fixedNext).normalize();

        // cross product, if >0 left turn, <0 right turn, =0 straight line
        double cross = in.getX() * out.getY() - in.getY() * out.getX();

        Vector outwardIn;
        Vector outwardOut;

        if (cross > 0) { // left turn
            outwardIn = new Vector(in.getY(), -in.getX()); // right normal vector (vector 90deg right)
            outwardOut = new Vector(out.getY(), -out.getX()); // right normal vector (vector 90deg right)
        } else if (cross < 0) { // right turn
            outwardIn = new Vector(-in.getY(), in.getX()); // left normal vector (vector 90 left)
            outwardOut = new Vector(-out.getY(), out.getX()); // left normal vector (vector 90 left)
        } else {
            return new Vector(0, 0); // straight line
        }

        double bx = outwardIn.getX() + outwardOut.getX();
        double by = outwardIn.getY() + outwardOut.getY();

        double mag = Math.sqrt(bx * bx + by * by);
        if (mag < EPS) { // if vector is extremely small it's basically 0
            return new Vector(0, 0);
        }

        return new Vector(bx / mag, by / mag); // unit vector that bisects the corner but points outward
    }

    private double getCornerWeight(Path path, int pointIndex) {
        if (pointIndex + 1 < path.numPoints()) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex + 1)) != -1) {
                return 1.0;
            }
        }
        if (pointIndex - 1 >= 0) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex - 1)) != -1) {
                return 1.0;
            }
        }

        if (pointIndex + 2 < path.numPoints()) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex + 2)) != -1) {
                return 0.5;
            }
        }
        if (pointIndex - 2 >= 0) {
            if (getOriginalWaypointIndex(path.getPoint(pointIndex - 2)) != -1) {
                return 0.5;
            }
        }

        return 0.0;
    }

    private void calculateDistanceAlongPath(Path path) {
        if (calcDistanceIndex <= path.numPoints()-1) {
            if (calcDistanceIndex == 0) {
                path.getPoint(calcDistanceIndex).setDistanceAlongPath(0);
            } else {
                Vector vector = Vector.between(path.getPoint(calcDistanceIndex - 1), path.getPoint(calcDistanceIndex));
                path.getPoint(calcDistanceIndex).setDistanceAlongPath(path.getPoint(calcDistanceIndex - 1).getDistanceAlongPath() + vector.getLength());

                KLog.d("ppDebug", () -> "set distance of point " + calcDistanceIndex + " to: " + (path.getPoint(calcDistanceIndex - 1).getDistanceAlongPath() + vector.getLength()));
            }

            calcDistanceIndex++;
        } else {
            calcDistanceDone = true;
        }
    }

    private void calculateVelocityAcceleration(Path path) {
        if (calcVAIndex < 0 && !calcVelocityAccelDone) {
            path.getPoint(path.numPoints() - 1).setCurvature(curvature(path, path.numPoints() - 1));
            path.getPoint(path.numPoints() - 1).setVelocity(0);
            path.getPoint(path.numPoints() - 1).setAcceleration(0);

            // Step 2: Set the loop index to start from the second-to-last point
            calcVAIndex = path.numPoints() - 2;
            KLog.d("adaptive pure pursuit velocity", () -> "set calcpoint to last point: " + calcVAIndex);
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

                double vPrev = path.getPoint(calcVAIndex -1).getVelocity();        // mm/s
                double sPrev = path.getPoint(calcVAIndex -1).getDistanceAlongPath(); // mm
                double vCurr = path.getPoint(calcVAIndex).getVelocity();          // mm/s
                double sCurr = path.getPoint(calcVAIndex).getDistanceAlongPath(); // mm

                KLog.d("ppDebug", () -> "set acceleration of point " + calcVAIndex + " using vPrev: " + vPrev + ", sPrev: " + sPrev+ ", vCurr: " + vCurr+ ", sCurr: " + sCurr);

                double deltaS = sCurr - sPrev;                          // mm
                // avoid divide‑by‑zero for back‑to‑back identical points
                double accelMmPerS2;
                if (Math.abs(deltaS) < 1e-6) {  // if points are essentially the same
                    accelMmPerS2 = 0;  // no acceleration needed
                } else {
                    accelMmPerS2 = (vCurr*vCurr - vPrev*vPrev) / (2.0 * deltaS);
                }
                // Clamp acceleration to reasonable bounds
                accelMmPerS2 = Math.max(-MAX_ACCELERATION, Math.min(MAX_ACCELERATION, accelMmPerS2));
                path.getPoint(calcVAIndex).setAcceleration(accelMmPerS2);
                double finalAccelMmPerS = accelMmPerS2;
                KLog.d("ppDebug", () -> "set acceleration of point " + calcVAIndex + " to: " + finalAccelMmPerS);

            }

            KLog.d("adaptive pure pursuit velocity", () -> "calculated point: " + calcVAIndex);
            calcVAIndex--;

        } else {
            calcVelocityAccelDone = true;
        }
    }

    public static double curvature(Path path, int positionIndex) {

        // Ensure that positionIndex allows for accessing points before and after
        if (positionIndex <= 0 || positionIndex >= path.numPoints() - 1) {
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
        // A positive curvature can indicate a turn to the left, and negative to the right, depending on your coordinate system and how "area2" is interpreted.
        // For pure pursuit, often the magnitude is used for speed, and steering is determined by the look-ahead point's position relative to the robot's heading.
        // However, if you want signed curvature, you can divide area2 by the radius components.
        // For general usage in path following, a simple 1/R is often sufficient, with the controller deciding the turn direction. If the original intent was signed curvature:
        return area2 / (radius * radius * 2); // This gives signed curvature more robustly.
        // If your system only needs positive curvature, use Math.abs(1/radius)
        // or just 1/radius if radius is always positive.

    }

    private double calculateVelocity(Path path, int positionIndex) {
        if (positionIndex <= 0) {
            return 0;                // don’t drive “backwards” into start
        } else if (positionIndex == path.numPoints()-1) {
            return 0;
        }

        double curvature = Math.abs(curvature(path, positionIndex));
        if (curvature < 1e-6) return PATH_MAX_VELOCITY; // straight line

        double mag = Math.min(PATH_MAX_VELOCITY, K / Math.abs(curvature(path, positionIndex)));

        return mag;
    }

    private double getTargetVelocity(Path path, int positionIndex) {
        // v_next is the velocity of the next point in the path, which is what we need to decelerate to.
        double v_next = path.getPoint(positionIndex + 1).getVelocity();

        double d = path.getPoint(positionIndex + 1).getDistanceAlongPath() - path.getPoint(positionIndex).getDistanceAlongPath();

        // calculate max velocity (v_f) at the current point to be able to decelerate to v_next over the distance d
        double v_f = Math.sqrt(MathFunctions.square(v_next) + (2 * MAX_ACCELERATION * d));

        KLog.d("ppDebug", () -> "velocity of " + positionIndex + " set to " + (Math.min(v_f, calculateVelocity(path, positionIndex)) == v_f ? "v_f" : "calculated vel"));
        //robot velocity at the current point is the minimum of max velocity allowed by the path's curvature, and max velocity allowed by deceleration to the next point.
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
        return (K_p * (wheelVelocity - filteredVelocityMmPerS) + K_v * wheelVelocity + K_a * Math.signum(wheelVelocity) * Math.abs(acceleration));
    }
}