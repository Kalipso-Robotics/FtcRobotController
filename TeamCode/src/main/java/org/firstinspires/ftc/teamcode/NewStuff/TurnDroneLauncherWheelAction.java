package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TurnDroneLauncherWheelAction extends Action {

    DcMotor wheel;
    final double ERROR_TOLERANCE = 20;
    final double P_CONSTANT = 0.004;
    Action dependentAction;
    DoneStateAction doneStateAction = new DoneStateAction();
    double targetTicks;
    double currentTicks;
    double error;
    boolean isDone = false;


    public TurnDroneLauncherWheelAction(Action dependentAction, double targetTicks, DroneLauncher droneLauncher) {
        wheel = droneLauncher.wheel;
        this.dependentAction = dependentAction;
        this.targetTicks = targetTicks;
    }

    public TurnDroneLauncherWheelAction(double targetTicks, DroneLauncher droneLauncher) {
        wheel = droneLauncher.wheel;
        this.dependentAction = doneStateAction;
        this.targetTicks = targetTicks;
    }

    //TODO make private
    public double calculatePower() {
        refreshError();
        return error * P_CONSTANT;
    }

    public void updateCheckDone() {
        Log.d("parallelaction", "entered turn drone wheel");
        if (isDone) { return; } //if i'm done never update
        if (!dependentAction.getIsDone()) { return; } //if dependent action is not done never update

        Log.d("parallelaction", "should turn drone wheel");
        update();

        updateIsDone();
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
    }

    @Override
    boolean updateIsDone() {
        refreshError();
        if (error <= ERROR_TOLERANCE) {
            isDone = true;
            return isDone;
        }
        isDone = false;
        return isDone;
    }

    @Override
    void update() {
        this.currentTicks = wheel.getCurrentPosition();
        wheel.setPower(calculatePower());
    }

    boolean getIsDone() {
        return isDone;
    }

    void setDependentAction(Action newAction) {
        this.dependentAction = newAction;
    }

    Action getDependentAction() {
        return this.dependentAction;
    }

}