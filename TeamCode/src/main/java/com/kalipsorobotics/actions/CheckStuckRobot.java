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

    private boolean checkRobotNotMoving(double currentXVelocity, double currentYVelocity, double currentTimeInMs) {
        if (currentYVelocity < 0.05 || currentXVelocity < 0.05) {
            return true;
        }
        if (getXDelta(SharedData.getOdometryPosition()) < X_DELTA_MIN_THRESHOLD && getYDelta(SharedData.getOdometryPosition()) < Y_DELTA_MIN_THRESHOLD) {
            return true;
        }
        return false;
    }
    private double prevHeading = 0;

    private double prevHeading = 0;

    private boolean checkRobotSpinning(double xDelta, double yDelta, double thetaDelta, Position currentPos, long currentTimeMs) {
        double currentHeading = currentPos.getTheta();
        double headingDelta = Math.abs(currentHeading - prevHeading);
        prevHeading = currentHeading;

        boolean isSpinningNow =
                Math.abs(xDelta) < X_DELTA_MIN_THRESHOLD &&
                        Math.abs(yDelta) < Y_DELTA_MIN_THRESHOLD &&
                        Math.abs(thetaDelta) < THETA_DELTA_MIN_THRESHOLD &&
                        headingDelta < 1;  // degrees

        if (isSpinningNow) {
            if (!wasSpinning) {
                wasSpinning = true;
                spinningStartTime = currentTimeMs;
            } else {
                if ((currentTimeMs - spinningStartTime) >= SPINNING_TIMEOUT_MS) {
                    return true;  // spinning "forever"
                }
            }
        } else {
            wasSpinning = false;
            spinningStartTime = 0;
        }

        return false;
    }


    private boolean checkIfOnPath(/*Path path,*/ int timeInMillis) {
        //return checkXY.isPositionOnPath(path, timeInMillis);
        return true;
    }

    // change from out of void when method finished
    // if delta x, y, and theta are too low ( make threshold large ) then check the path and current pos
    public boolean isStuck(/*Path path,*/ int timeInMillis) {
        Position currentPos = SharedData.getOdometryPosition();
        Position intendedPos = currentPos;

        //TODO add intended pos
        double currentX = currentPos.getX();
        double currentY = currentPos.getY();
        double currentTheta = currentPos.getTheta();

//        double currentThetaVelocity = wheelOdometry.getCurrentVelocity().getTheta();
//        double deltaThetaVelocity = abs(prevThetaVelocity - currentThetaVelocity);
//
//        double currentYVelocity = wheelOdometry.getCurrentVelocity().getY();
//        double deltaYVelocity = abs(prevYVelocity - currentYVelocity);
//
//        double currentXVelocity = wheelOdometry.
//        double deltaXVelocity = abs(prevXVelocity - currentXVelocity);

        //TODO Fix the time encrements
        // Declare and initialize this somewhere outside the method, so it persists across calls:
// long lastStuckCheckTime = 0;

        if (timeInMillis - lastStuckCheckTime >= 1000) {
            lastStuckCheckTime = timeInMillis;  // reset the timer

            if (/*checkRobotSpinning(
                    getXDelta(currentPos),
                    getYDelta(currentPos),
                    getThetaDelta(currentPos),
                    currentPos,
                    currentPos.getTheta(),
                    timeInMillis) ||*/
                    checkRobotNotMoving(
                            getXDelta(currentPos),
                            getYDelta(currentPos),
                            timeInMillis)) {

                Log.d("check stuck", "---ROBOT IS STUCK---");
                if (checkRobotSpinning(
                        getXDelta(currentPos),
                        getYDelta(currentPos),
                        getThetaDelta(currentPos),
                        currentPos,
                        timeInMillis)) {
                    //robot is spinning, unstuck
                }
                if (checkRobotNotMoving(
                        getXDelta(currentPos),
                        getYDelta(currentPos),
                        timeInMillis)) {
                    //move backwards to unstuck
                }
                return true;
            }

            Log.d("check stuck", "---robot is not stuck---");
        }

        return false;

    }

    private void unstuckRobot(DriveTrain driveTrain, int timeInMillis){
        purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.setMaxTimeOutMS(500);
        Position currentPos = new Position(wheelOdometry.countLeft(), wheelOdometry.countBack(), wheelOdometry.getCurrentImuHeading());
        //TODO replace 10 with something
        Position possiblePos1 = new Position(currentPos.getX() - 10, currentPos.getY(), currentPos.getTheta());
        Position possiblePos2 = new Position(currentPos.getX() + 10, currentPos.getY(), currentPos.getTheta());
        Position possiblePos3 = new Position(currentPos.getX(), currentPos.getY() - 10, currentPos.getTheta());
        Position possiblePos4 = new Position(currentPos.getX(), currentPos.getY() + 10, currentPos.getTheta());
        if (!checkIfOnPath(/*path*/timeInMillis)) {
            purePursuitAction.addPoint(possiblePos1.getX(), possiblePos1.getY(), possiblePos1.getTheta());
            purePursuitAction.addPoint(possiblePos2.getX(), possiblePos2.getY(), possiblePos2.getTheta());
            purePursuitAction.addPoint(possiblePos3.getX(), possiblePos3.getY(), possiblePos3.getTheta());
            purePursuitAction.addPoint(possiblePos4.getX(), possiblePos4.getY(), possiblePos4.getTheta());
            return;
        }
        //moves in all four directions
        //TODO replace with something that actually gets the robot unstuck
        return;
    }

}
