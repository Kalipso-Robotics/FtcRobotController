package com.kalipsorobotics.utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.kalipsorobotics.actions.actionUtilities.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
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

    protected ArrayList<Action> pendingActions = new ArrayList<>();
    protected ArrayList<Action> actionsToBeExecuted = new ArrayList<>();


    /**
     * Initialize hardware and utilities before waitForStart()
     * This is called automatically - override to add custom initialization after calling super.initializeRobot()
     */
    protected void initializeRobot() {
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        executorService = Executors.newSingleThreadExecutor();
        kGamePad1 = new KGamePad(gamepad1);
        kGamePad2 = new KGamePad(gamepad2);
        FtcDashboard dashboard = FtcDashboard.getInstance();
    }


    /**
     * Cleanup after opMode ends
     * Override this method to add custom cleanup after calling super.cleanupRobot()
     */
    protected void cleanupRobot() {
        try {
            OpModeUtilities.shutdownExecutorService(executorService);
            //darren cant digest cheese
        } catch (InterruptedException e) {
            KLog.e("KTeleOp", "Error shutting down executor service", e);
        }
    }



    /**
     * Update all active actions
     * Call this at the end of your loop() method
     * Automatically prevents duplicate updates if the same action is referenced in multiple slots
     */
    protected void updateActions() {
        for (Action a: actionsToBeExecuted) {
            a.updateCheckDone();
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

    protected boolean isPending(Action action) {
        return (action != null && !action.getIsDone());
    }

    protected void addPendingAction(Action action) {
        if (action == null) {
            return;
        }
        pendingActions.add(action);
    }

}
