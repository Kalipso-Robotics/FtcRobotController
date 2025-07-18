package com.kalipsorobotics.actions;
import static java.lang.Math.abs;

import android.app.SharedElementCallback;
import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.test.checkStuck.CheckXY;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class CheckStuckRobot {
    private double prevXPos = 0;
    private double prevYPos = 0;
    private double prevThetaPos = 0;
    private boolean wasSpinning = false;
    private boolean wasNotMovingLastCycle = false;
    private double spinningStartTime = 0;
    private static final long SPINNING_TIMEOUT_MS = 2000; // 2 seconds
    private long lastStuckCheckTime;
    private double prevXVelocity = 0;
    private double timeInMsSinceLastChecked;
    private double timeOffset;
    /**
     * mm per second
     * */
    final private double X_DELTA_MIN_THRESHOLD = 1; // to be calc
    private double prevYVelocity = 0;
    final private double Y_DELTA_MIN_THRESHOLD = 1; // to be calc

    private double prevThetaVelocity = 0;

    final private double THETA_DELTA_MIN_THRESHOLD = 1; // to be calc
    private double fLeftPower;
    private double fRightPower;
    private double bLeftPower;
    private double bRightPower;
    private final WheelOdometry wheelOdometry;
    private final DriveTrain driveTrain;
    private final OpModeUtilities opModeUtilities;
    private PurePursuitAction purePursuitAction;
    private CheckXY checkXY;

    public CheckStuckRobot(DriveTrain driveTrain, WheelOdometry wheelOdometry, OpModeUtilities opModeUtilities, PurePursuitAction purePursuitAction){
        this.wheelOdometry = wheelOdometry;
        this.driveTrain = driveTrain;
        this.opModeUtilities = opModeUtilities;
        this.purePursuitAction = purePursuitAction;
        checkXY = new CheckXY(opModeUtilities);
        timeInMsSinceLastChecked = 0;
        timeOffset = 0;
        lastStuckCheckTime = 0;
    }

    public double getXDelta(Position currentPosition) {
        double currentxDelta = currentPosition.getX() - prevXPos;
        prevXPos = currentPosition.getX();
        return currentxDelta;
    }
    public double getYDelta(Position currentPosition) {
        double currentyDelta = currentPosition.getY() - prevYPos;
        prevYPos = currentPosition.getY();
        return currentyDelta;
    }
    public double getThetaDelta(Position currentPosition) {
        double currentThetaDelta = currentPosition.getTheta() - prevThetaPos;
        prevThetaPos = currentPosition.getTheta();
        return currentThetaDelta;
    }

//    private boolean isXDeltaValid() {
//        double currentXVelocity = wheelOdometry.getCurrentVelocity().getX();
//        double deltaXVelocity = abs(prevXVelocity - currentXVelocity);
//
//        if (deltaXVelocity < X_DELTA_MIN_THRESHOLD) {
//            return true;
//        }
//        return false;
//    }
//
//    private boolean isYDeltaValid() {
//        double currentYVelocity = wheelOdometry.getCurrentVelocity().getY();
//        double deltaYVelocity = abs(prevYVelocity - currentYVelocity);
//
//        if (deltaYVelocity < Y_DELTA_MIN_THRESHOLD) {
//            return true;
//        }
//        return false;
//    }
//
//    private boolean isThetaDeltaValid() {
//        double currentThetaVelocity = wheelOdometry.getCurrentVelocity().getTheta();
//        double deltaThetaVelocity = abs(prevThetaVelocity - currentThetaVelocity);
//
//        if (deltaThetaVelocity < THETA_DELTA_MIN_THRESHOLD) {
//            return true;
//        }
//        return false;
//    }
//    private boolean checkDeltaValid() {
//        if (isThetaDeltaValid() && isXDeltaValid() && isYDeltaValid()) {
//            return true;
//        }
//        return false;
//    }

    private boolean checkRobotNotMoving(double xDelta, double yDelta) {
        if (driveTrain.getbLeft().getPower() > 0 || driveTrain.getfLeft().getPower() > 0 || driveTrain.getfRight().getPower() > 0 || driveTrain.getbRight().getPower() > 0) {
            return (Math.abs(xDelta) < X_DELTA_MIN_THRESHOLD && Math.abs(yDelta) < Y_DELTA_MIN_THRESHOLD); //making suer the robot is trying to move
        } else {
            return false;
        }
    }

    private double prevHeading = 0;


    private boolean checkRobotSpinning(double xDelta, double yDelta, double thetaDelta, Position currentPos, long currentTimeMs) {
        double currentHeading = currentPos.getTheta();
        double headingDelta = Math.abs(currentHeading - prevHeading);

        boolean isCurrentlySpinning =
                Math.abs(xDelta) < X_DELTA_MIN_THRESHOLD &&
                        Math.abs(yDelta) < Y_DELTA_MIN_THRESHOLD &&
                        Math.abs(thetaDelta) < THETA_DELTA_MIN_THRESHOLD &&
                        headingDelta < 1; // degrees


        if (isCurrentlySpinning) {
            if (!wasSpinning) {
                wasSpinning = true;
                spinningStartTime = currentTimeMs;
            } else if (currentTimeMs - spinningStartTime >= SPINNING_TIMEOUT_MS) {
                prevHeading = currentHeading;  // <- only update heading when actually spinning
                return true;  // spinning too long = stuck
            }
        } else {
            wasSpinning = false;
            spinningStartTime = 0;
        }

        prevHeading = currentHeading;  // move to end to avoid race conditions

        return false;
    }



    private boolean checkIfOnPath(/*Path path,*/ int timeInMillis) {
        //return checkXY.isPositionOnPath(path, timeInMillis);
        return true;
    }

    // change from out of void when method finished
    // if delta x, y, and theta are too low ( make threshold large ) then check the path and current pos
    public boolean isStuck(Position currentPos) {
        long currentTime = SystemClock.uptimeMillis();
        Position currentPosition = currentPos;
        double xDelta = getXDelta(currentPosition);
        double yDelta = getYDelta(currentPosition);
        double thetaDelta = getThetaDelta(currentPosition);

        boolean spinning = checkRobotSpinning(xDelta, yDelta, thetaDelta, currentPosition, currentTime);
        boolean notMoving = checkRobotNotMoving(xDelta, yDelta);

        if (currentTime - lastStuckCheckTime >= 1000) {
            lastStuckCheckTime = currentTime;

            if (notMoving || spinning) {
                bLeftPower = driveTrain.getbLeft().getPower();
                fLeftPower = driveTrain.getfLeft().getPower();
                bRightPower = driveTrain.getbRight().getPower();
                fRightPower = driveTrain.getfRight().getPower();
//                if (bLeftPower > 0 || fLeftPower > 0 || bRightPower > 0 || fRightPower > 0) {
//
//                }
                Log.d("check stuck", "---ROBOT IS STUCK---");
                unstuckRobot(driveTrain, currentPosition);
                return true;
            }

            Log.d("check stuck", "---robot is not stuck---");
        }

        return false;
    }




    private void unstuckRobot(DriveTrain driveTrain, Position currentPos) {
        purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.setMaxTimeOutMS(500);

        Position currentPosition = currentPos;


        double offset = 100; // distance in mm to try moving

        Position[] escapePositions = new Position[] {
                new Position(currentPosition.getX() - offset, currentPosition.getY(), currentPosition.getTheta()), // move left
                new Position(currentPosition.getX() + offset, currentPosition.getY(), currentPosition.getTheta()), // move right
                new Position(currentPosition.getX(), currentPosition.getY() - offset, currentPosition.getTheta()), // move backward
                new Position(currentPosition.getX(), currentPosition.getY() + offset, currentPosition.getTheta())  // move forward
        };

        // Add all directions to path (could refine this to pick based on space later)
        for (Position escapePos : escapePositions) {
            purePursuitAction.addPoint(escapePos.getX(), escapePos.getY(), escapePos.getTheta());
        }

        Log.d("unstuck", "Trying to move in multiple directions to escape.");
    }



}
