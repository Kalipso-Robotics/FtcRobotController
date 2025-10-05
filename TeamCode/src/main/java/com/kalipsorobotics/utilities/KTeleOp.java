package com.kalipsorobotics.utilities;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Base class for TeleOp modes that handles common initialization and action management
 */
public abstract class KTeleOp extends LinearOpMode {

    protected OpModeUtilities opModeUtilities;
    protected ExecutorService executorService;
    protected KGamePad kGamePad1;
    protected KGamePad kGamePad2;

    // Action tracking - only one action per category can be active at a time
    protected Action lastIntakeAction = null;
    protected Action lastMoveAction = null;
    protected Action lastShooterAction = null;

    protected Action lastKickerAction = null;

    /**
     * Initialize hardware and utilities before waitForStart()
     * This is called automatically - override to add custom initialization after calling super.initializeRobot()
     */
    protected void initializeRobot() {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        executorService = Executors.newSingleThreadExecutor();
        kGamePad1 = new KGamePad(gamepad1);
        kGamePad2 = new KGamePad(gamepad2);
    }


    /**
     * Cleanup after opMode ends
     * Override this method to add custom cleanup after calling super.cleanupRobot()
     */
    protected void cleanupRobot() {
        try {
            OpModeUtilities.shutdownExecutorService(executorService);
        } catch (InterruptedException e) {
            Log.e("KTeleOp", "Error shutting down executor service", e);
        }
    }



    /**
     * Set the last intake action and update it
     * Cancels any previous intake action
     * @param action the new action to set, or null to cancel current action
     */
    protected void setLastIntakeAction(Action action) {
        lastIntakeAction = action;
    }

    /**
     * Set the last move action and update it
     * Cancels any previous move action
     * @param action the new action to set, or null to cancel current action
     */
    protected void setLastMoveAction(Action action) {
        lastMoveAction = action;
    }

    /**
     * Set the last shooter action and update it
     * Cancels any previous shooter action
     * @param action the new action to set, or null to cancel current action
     */
    protected void setLastShooterAction(Action action) {
        lastShooterAction = action;
    }

    protected void setLastKickerAction(Action action) {
        lastKickerAction = action;
    }

    /**
     * Update all active actions
     * Call this at the end of your loop() method
     */
    protected void updateActions() {
        if (lastIntakeAction != null) {
            lastIntakeAction.updateCheckDone();
        }

        if (lastMoveAction != null) {
            lastMoveAction.updateCheckDone();
        }

        if (lastShooterAction != null) {
            lastShooterAction.updateCheckDone();
        }

        if (lastKickerAction != null) {
            lastKickerAction.updateCheckDone();
        }
    }

    /**
     * Check if gamepad drive joystick is at zero position
     * @return true if all drive joystick inputs are zero
     */
    protected boolean isGamePadDriveJoystickZero() {
        return gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0;
    }

    /**
     * Check if gamepad drive joystick is being used
     * @return true if any drive joystick input is non-zero
     */
    protected boolean isDriving() {
        return !isGamePadDriveJoystickZero();
    }
}
