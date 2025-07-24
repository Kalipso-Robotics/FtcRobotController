package com.kalipsorobotics.actions.drivetrain;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.DriveTrain;

public class MecanumRobotAction extends DriveTrainAction {

    private static final double ERROR_TOLERANCE_IN = 0.5;
    private static final double HEADING_ERROR_TOLERANCE_DEG = 2;
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

    double overshoot;
    private double initialError;

    public MecanumRobotAction(double targetInches, DriveTrain driveTrain, SparkfunOdometry sparkfunOdometry, WheelOdometry wheelOdometry, double targetTheta, double timeout) {
        this.dependentActions.add(new DoneStateAction());
        this.pidController = new PIDController(0.093334, 0.000349, 0.002371, "mecanum");
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
        this.pidController = controller;
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

    private void refreshThetaOffset() {thetaOffset = targetTheta - wheelOdometry.getCurrentImuHeading();}

    @Override
    public boolean checkDoneCondition() {
        refreshRemainingDistance();
        duration = (SystemClock.elapsedRealtime() - startTime) / 1000;
        if ((Math.abs(remainingDistance) <= ERROR_TOLERANCE_IN && Math.abs(thetaOffset) <= Math.toRadians(HEADING_ERROR_TOLERANCE_DEG)) || duration > timeout) {
            driveTrain.setPower(0); // stop, to be safe
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            Log.d("mecanum", "done");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        double minPower = 0.2;

        this.currentInches = wheelOdometry.countBack() / 25.4;
        if (!hasStarted) {
            this.targetInches += currentInches;
            initialError = targetInches - currentInches;
            Log.d("mecanum","target inches is " + this.targetInches);

            hasStarted = true;
            startTime = SystemClock.elapsedRealtime();
        }

        refreshRemainingDistance();
        refreshThetaOffset();

        Log.d("mecanum/error", String.format("Error %f Current Inches %f Target Inches %f", remainingDistance, currentInches, targetInches));

        double strafePower = pidController.calculate(remainingDistance);
        double rotationPower = 0;

        if (Math.abs(thetaOffset) > Math.toRadians(HEADING_ERROR_TOLERANCE_DEG)) {
            if (thetaOffset < 0) {
                rotationPower = Math.min(0.05, thetaOffset * 0.4);
            } else {
                rotationPower = Math.max(0.05, thetaOffset * 0.4);

            }
        }

        double fLeft = strafePower + rotationPower;
        double fRight = -strafePower - rotationPower;
        double bLeft = -strafePower + rotationPower;
        double bRight = strafePower - rotationPower;

        double biggest = MathFunctions.maxAbsValueDouble(fLeft, fRight, bLeft, bRight);
        double smallest = MathFunctions.minAbsValueDouble(fLeft, fRight, bLeft, bRight);
        double scalingFactor = 1;

//        Log.d("mecanum/range", biggest + " " + smallest);

        if (Math.abs(smallest) < minPower) {
            scalingFactor = minPower / smallest;
        }

        fLeft *= scalingFactor;
        fRight *= scalingFactor;
        bLeft *= scalingFactor;
        bRight *= scalingFactor;

        if (biggest > 1) {
            fLeft /= biggest;
            fRight /= biggest;
            bLeft /= biggest;
            bRight /= biggest;
        }

        Log.d("mecanum/powers/strafe", String.format("Strafe power %f", strafePower));
        Log.d("mecanum/powers/rotation", String.format("Theta offset %f, Rotation Power %f", thetaOffset, rotationPower));
        Log.d("mecanum/powers/all", String.format("fLeft %f fRight %f bLeft %f bRight %f", fLeft, fRight, bLeft, bRight));

        driveTrain.setPower(fLeft, fRight, bLeft, bRight);
    }
}
