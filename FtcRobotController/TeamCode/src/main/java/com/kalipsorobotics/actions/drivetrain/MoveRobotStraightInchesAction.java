package com.kalipsorobotics.actions.drivetrain;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveRobotStraightInchesAction extends DriveTrainAction {
    private static final double ERROR_TOLERANCE_IN = 0.5;
    private static final double HEADING_ERROR_TOLERANCE_DEG = 1;
    DriveTrain driveTrain;
    SparkfunOdometry sparkfunOdometry;
    WheelOdometry wheelOdometry;
    PIDController pidController;

    double targetInches;
    double currentInches;
    double remainingDistance;

    double targetTheta;
    double thetaOffset;

    double startTime;
    double timeout;
    double duration;

    double overshoot = 0.;
    private double initialError = 0;

    public MoveRobotStraightInchesAction(double targetInches, DriveTrain driveTrain, SparkfunOdometry sparkfunOdometry, WheelOdometry wheelOdometry, double targetTheta, double timeout) {
        this.dependentActions.add(new DoneStateAction());
        this.pidController = new PIDController(0.079376, 0.000394, 0.008647, "straight");
        this.driveTrain = driveTrain;

        this.sparkfunOdometry = sparkfunOdometry;
        this.wheelOdometry = wheelOdometry;

        this.targetInches = targetInches;
        this.targetTheta = targetTheta;

        this.startTime = Integer.MAX_VALUE;
        this.timeout = timeout;
    }

    public PIDController getPidController() {
        return pidController;
    }

    public void setPidController(PIDController controller) {
        pidController = controller;
    }

    public double getRemainingDistance() {
        return remainingDistance;
    }

    public double getTarget() {
        return targetInches;
    }

    public double getDuration() {
        return duration;
    }

    public double getOvershoot() {
        return overshoot;
    }

    private void refreshRemainingDistance() {
        remainingDistance = targetInches - currentInches;
        if (Math.signum(remainingDistance) != Math.signum(initialError)) {
            overshoot = Math.abs(remainingDistance);
        }
    }

    private void refreshThetaOffset() {
        thetaOffset = targetTheta - wheelOdometry.getCurrentImuHeading();
    }

    @Override
    public boolean checkDoneCondition() {
        refreshRemainingDistance();
        duration = (SystemClock.elapsedRealtime() - startTime) / 1000;
        if ((Math.abs(remainingDistance) <= ERROR_TOLERANCE_IN && Math.abs(thetaOffset) <= Math.toRadians(HEADING_ERROR_TOLERANCE_DEG)) || duration > timeout) {
            driveTrain.setPower(0);
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            Log.d("straight", "isdone true");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        double minPower = 0.1;

        this.currentInches = wheelOdometry.countLeft() / 25.4;
        if (!hasStarted) {
            this.targetInches += currentInches;
            initialError = targetInches - currentInches;
            Log.d("straight", "target inches is " + this.targetInches);

            hasStarted = true;
            startTime = SystemClock.elapsedRealtime();
        }

        refreshRemainingDistance();
        refreshThetaOffset();

        Log.d("straight/error", String.format("Error %f Current Inches %f Target Inches %f", remainingDistance, currentInches, targetInches));

        double linearPower = pidController.calculate(remainingDistance);
        double rotationPower = 0;

        if (Math.abs(thetaOffset) > Math.toRadians(HEADING_ERROR_TOLERANCE_DEG)) {
            rotationPower = thetaOffset * 0.25;
        }

        double fLeft = linearPower + rotationPower;
        double fRight = linearPower - rotationPower;
        double bLeft = linearPower + rotationPower;
        double bRight = linearPower - rotationPower;

        double biggest = MathFunctions.maxAbsValueDouble(fLeft, fRight, bLeft, bRight);
        double smallest = MathFunctions.minAbsValueDouble(fLeft, fRight, bLeft, bRight);

        if (smallest < minPower) {
            fLeft *= (minPower / smallest);
            fRight *= (minPower / smallest);
            bLeft *= (minPower / smallest);
            bRight *= (minPower / smallest);
        }
        if (biggest > 1) {
            fLeft /= biggest;
            fRight /= biggest;
            bLeft /= biggest;
            bRight /= biggest;
        }

        Log.d("straight/powers/linear", String.format("Linear power %f", linearPower));
        Log.d("straight/powers/rotation", String.format("Theta offset %f, Rotation Power %f", thetaOffset, rotationPower));
        Log.d("straight/powers/all", String.format("fLeft %f fRight %f bLeft %f bRight %f", fLeft, fRight, bLeft, bRight));

        driveTrain.setPower(fLeft, fRight, bLeft, bRight);
    }
}
