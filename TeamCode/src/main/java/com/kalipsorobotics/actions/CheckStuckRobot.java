package com.kalipsorobotics.actions;
import static java.lang.Math.abs;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.test.checkStuck.CheckXY;
import com.kalipsorobotics.utilities.OpModeUtilities;

import java.util.ArrayList;
import java.util.List;

public class CheckStuckRobot {
    private double prevXPos = 0;
    private double prevYPos = 0;
    private double prevThetaPos = 0;
    private boolean wasSpinning = false;
    private double spinningStartTime = 0;
    private static final long SPINNING_TIMEOUT_MS = 2000; // 2 seconds
    private long lastStuckCheckTime;

    /**
     * mm per second
     */
    final private double X_DELTA_MIN_THRESHOLD = 1; // to be calc
    final private double Y_DELTA_MIN_THRESHOLD = 1; // to be calc
    final private double THETA_DELTA_MIN_THRESHOLD = 1; // to be calc
    final private double HEADING_DELTA_THRESHOLD = 1; // degrees - minimum change to detect spinning

    // Position history for reverse navigation
    private List<Position> positionHistory = new ArrayList<>();
    private static final int MAX_HISTORY_SIZE = 50; // Keep last 50 positions
    private static final long POSITION_UPDATE_INTERVAL_MS = 500; // Update position every 500ms
    private Position lastSafePosition = null;
    private boolean isCurrentlyStuck = false;
    private long lastPositionUpdateTime = 0;

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
        lastStuckCheckTime = 0;
    }

    public double getXDelta(Position currentPosition) {
        if (currentPosition == null) {
            return 0;
        }
        double currentXDelta = currentPosition.getX() - prevXPos;
        prevXPos = currentPosition.getX();
        return currentXDelta;
    }

    public double getYDelta(Position currentPosition) {
        if (currentPosition == null) {
            return 0;
        }
        double currentYDelta = currentPosition.getY() - prevYPos;
        prevYPos = currentPosition.getY();
        return currentYDelta;
    }

    public double getThetaDelta(Position currentPosition) {
        if (currentPosition == null) {
            return 0;
        }
        double currentThetaDelta = currentPosition.getTheta() - prevThetaPos;
        prevThetaPos = currentPosition.getTheta();
        return currentThetaDelta;
    }

    private boolean checkRobotNotMoving(double xDelta, double yDelta) {
        if (driveTrain.getbLeft().getPower() > 0 || driveTrain.getfLeft().getPower() > 0 ||
                driveTrain.getfRight().getPower() > 0 || driveTrain.getbRight().getPower() > 0) {
            return (Math.abs(xDelta) < X_DELTA_MIN_THRESHOLD && Math.abs(yDelta) < Y_DELTA_MIN_THRESHOLD); //making sure the robot is trying to move
        } else {
            return false;
        }
    }

    private double prevHeading = 0;

    private boolean checkRobotSpinning(double xDelta, double yDelta, double thetaDelta, Position currentPos, long currentTimeMs) {
        if (currentPos == null) {
            return false;
        }

        double currentHeading = currentPos.getTheta();
        double headingDelta = Math.abs(currentHeading - prevHeading);

        // Robot is spinning if it's not moving in X/Y but has significant heading change
        boolean isCurrentlySpinning =
                Math.abs(xDelta) < X_DELTA_MIN_THRESHOLD &&
                        Math.abs(yDelta) < Y_DELTA_MIN_THRESHOLD &&
                        Math.abs(thetaDelta) < THETA_DELTA_MIN_THRESHOLD &&
                        headingDelta > HEADING_DELTA_THRESHOLD; // degrees - should be GREATER than threshold

        if (isCurrentlySpinning) {
            if (!wasSpinning) {
                wasSpinning = true;
                spinningStartTime = currentTimeMs;
            } else if (currentTimeMs - spinningStartTime >= SPINNING_TIMEOUT_MS) {
                return true;  // spinning too long = stuck
            }
        } else {
            wasSpinning = false;
            spinningStartTime = 0;
        }

        prevHeading = currentHeading;

        return false;
    }

    private boolean checkIfOnPath(int timeInMillis) {
        //return checkXY.isPositionOnPath(path, timeInMillis);
        return true;
    }

    /**
     * Add current position to history every 500ms
     */
    private void updatePositionHistory(Position currentPos) {
        if (currentPos == null) {
            return;
        }

        long currentTime = SystemClock.uptimeMillis();

        // Update position history every 500ms
        if (currentTime - lastPositionUpdateTime >= POSITION_UPDATE_INTERVAL_MS) {
            positionHistory.add(currentPos);
            lastPositionUpdateTime = currentTime;

            // Keep history size manageable
            if (positionHistory.size() > MAX_HISTORY_SIZE) {
                positionHistory.remove(0);
            }

            // Update last safe position if we're not currently stuck
            if (!isCurrentlyStuck) {
                lastSafePosition = currentPos;
            }
        }
    }

    /**
     * Find the best safe position to reverse to
     */
    public Position findBestSafePosition(Position currentPos) {
        if (lastSafePosition == null || positionHistory.isEmpty()) {
            return null;
        }

        // Start from the most recent safe position and work backwards
        for (int i = positionHistory.size() - 1; i >= 0; i--) {
            Position candidate = positionHistory.get(i);

            // Skip if it's too close to current position
            double distance = Math.sqrt(
                    Math.pow(currentPos.getX() - candidate.getX(), 2) +
                            Math.pow(currentPos.getY() - candidate.getY(), 2)
            );

            if (distance > 50) { // At least 50mm away
                return candidate;
            }
        }

        return lastSafePosition;
    }

    // change from out of void when method finished
    // if delta x, y, and theta are too low ( make threshold large ) then check the path and current pos
    public boolean isStuck(Position currentPos) {
        if (currentPos == null) {
            return false;
        }

        // Update position history
        updatePositionHistory(currentPos);

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
                isCurrentlyStuck = true;
                Log.d("check stuck", "---ROBOT IS STUCK---");
                unstuckRobot(driveTrain, currentPosition);
                return true;
            } else {
                isCurrentlyStuck = false;
                Log.d("check stuck", "---robot is not stuck---");
            }
        }

        return false;
    }





    private void unstuckRobot(DriveTrain driveTrain, Position currentPos) {
        if (currentPos == null) {
            return;
        }

        purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.setMaxTimeOutMS(2000); // Give more time for reverse navigation

        Position currentPosition = currentPos;
        Position safePosition = findBestSafePosition(currentPosition);

        if (safePosition != null) {
            // Use pure pursuit to reverse to the safe position
            Log.d("unstuck", "Reversing to safe position: (" + safePosition.getX() + ", " + safePosition.getY() + ")");
            purePursuitAction.addPoint(safePosition.getX(), safePosition.getY(), safePosition.getTheta());

            // Add a few intermediate points for smoother navigation
            double dx = safePosition.getX() - currentPosition.getX();
            double dy = safePosition.getY() - currentPosition.getY();
            double distance = Math.sqrt(dx * dx + dy * dy);

            if (distance > 100) { // Only add intermediate points for longer distances
                // Add midpoint
                double midX = currentPosition.getX() + dx * 0.5;
                double midY = currentPosition.getY() + dy * 0.5;
                purePursuitAction.addPoint(midX, midY, safePosition.getTheta());
            }
        } else {
            // Fallback to original escape strategy if no safe position found
            Log.d("unstuck", "No safe position found, using escape strategy");
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
        }

        Log.d("unstuck", "Trying to reverse to safe position or escape.");
    }
}