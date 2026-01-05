package com.kalipsorobotics.utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.ResetOdometryToPosition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Base class for TeleOp modes that handles common initialization and action management
 */
public abstract class KOpMode extends LinearOpMode {

    protected OpModeUtilities opModeUtilities;
    protected ExecutorService executorService;
    protected KGamePad kGamePad1;
    protected KGamePad kGamePad2;
    protected AllianceColor allianceColor = AllianceColor.RED; //defaults to red
    protected Action lastIntakeAction = null;
    protected Action lastMoveAction = null;
    protected Action lastShooterAction = null;
    protected Action lastBrakingAction = null;
    protected Action lastStopperAction = null;
    protected TurretAutoAlignTeleOp turretAutoAlignTeleOp = null;
    protected ShooterRun shooterRun = null;
    protected ResetOdometryToPosition resetOdometryToPosition = null;



    /**
     * Initialize hardware and utilities before waitForStart()
     * This is called automatically - override to add custom initialization after calling super.initializeRobot()
     */
    protected void initializeRobot() {
        initializeRobotConfig();
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        executorService = Executors.newSingleThreadExecutor();
        kGamePad1 = new KGamePad(gamepad1);
        kGamePad2 = new KGamePad(gamepad2);
        FtcDashboard dashboard = FtcDashboard.getInstance();
    }

    protected void initializeRobotConfig() {
    }

    /**
     * Cleanup after opMode ends
     * Override this method to add custom cleanup after calling super.cleanupRobot()
     */
    protected void cleanupRobot() {
        try {
            KLog.d("CleanupRobot", "Executor service shutdown started. Executor Service: " + executorService);
            OpModeUtilities.shutdownExecutorService(executorService);
            //darren cant digest cheese
        } catch (InterruptedException e) {
            // InterruptedException is expected during shutdown - restore interrupt status
            KLog.d("KOpMode", "Executor service shutdown interrupted (normal during stop)");
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
    protected void setLastBrakingAction(Action action) {
        lastBrakingAction = action;
    }
    protected void setLastStopperAction(Action action) {
        lastStopperAction = action;
    }

    /**
     * Update all active actions
     * Call this at the end of your loop() method
     * Automatically prevents duplicate updates if the same action is referenced in multiple slots
     */
    protected void updateActions() {
        if (lastIntakeAction != null) {
            lastIntakeAction.updateCheckDone();
        }

        if (lastMoveAction != null && lastMoveAction != lastIntakeAction) {
            lastMoveAction.updateCheckDone();
        }

        if (lastShooterAction != null && lastShooterAction != lastIntakeAction && lastShooterAction != lastMoveAction) {
            lastShooterAction.updateCheckDone();
        }

        if (lastStopperAction != null && lastStopperAction != lastIntakeAction && lastStopperAction != lastMoveAction && lastStopperAction != lastShooterAction) {
            KLog.d("KOpMode", "lastStopperAction updateCheckDone");
            lastStopperAction.updateCheckDone();
        }

        if (lastBrakingAction != null) {
            lastBrakingAction.updateCheckDone();
        }

        if (shooterRun != null) {
            shooterRun.updateCheckDone();
        }

        if (resetOdometryToPosition != null) {
            resetOdometryToPosition.updateCheckDone();
        }
        
        if (turretAutoAlignTeleOp != null) {
            turretAutoAlignTeleOp.updateCheckDone();
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

}
