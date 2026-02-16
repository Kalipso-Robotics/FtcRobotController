package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveAction {
    OpModeUtilities opModeUtilities;

    DriveTrain driveTrain;
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final DcMotor backEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;

    // --------------- FOR MOVE VELOCITY --------------
    private Position prevPosition = null;
    private final double lastMilli = 0;
    private final double MAX_WHEEL_VELOCITY_MM_S = 0.0; //todo measure straight max velocity
    private final double MAX_X_VELOCITY_MM_S = 0.8 * MAX_WHEEL_VELOCITY_MM_S; //scale down to not oversaturate power
    private final double MAX_Y_VELOCITY_MM_S = 0.9; //todo measure strafe max velocity
    private final double MAX_ANGLE_VELOCITY_RAD_S = 0.9 * 5.5; //todo remeasure on this year robot
    private final double K_px = PurePursuitAction.P_XY;
    private final double K_py = PurePursuitAction.P_XY;
    private final double K_pTheta = PurePursuitAction.P_ANGLE;
    private final double FILTER_SMOOTHING_FACTOR = 0.25;
    private double filteredVXCurrent = 0;
    private double filteredVYCurrent = 0;
    private double filteredVThetaCurrent = 0;



    private double powerCoefficient = 1;

    private final double WHEELBASE_LENGTH = 300; //front wheel to back wheel
    private double startTimeMS = System.currentTimeMillis();
    private boolean hasStarted = false;
    // -----------------------------------------------

    private final double[] driveTrainPower = new double[4];

    public DriveAction(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.fLeft = driveTrain.getfLeft();
        this.fRight = driveTrain.getfRight();
        this.bLeft = driveTrain.getbLeft();
        this.bRight = driveTrain.getbRight();
        this.rightEncoder = driveTrain.getRightEncoder();
        this.leftEncoder = driveTrain.getLeftEncoder();
        this.backEncoder = driveTrain.getBackEncoder();
    }

    public double[] calculatePower1(double gamepadx, double gamepady, Gamepad gamepad) {
        //negative because gamepad y is flip
        double forward = -gamepady * -gamepady * -gamepady; //cube so slower is slower and faster is faster
        double turn = gamepad.right_stick_x * gamepad.right_stick_x * gamepad.right_stick_x;
        double strafe = gamepadx * gamepadx * gamepadx;

        KLog.d("drive", "forward " + forward);
        KLog.d("drive", "turn " + turn);
        KLog.d("drive", "strafe " + strafe);

        double fLeftPower = (forward + strafe + turn);
        double fRightPower = (forward - strafe - turn);
        double bLeftPower = (forward - strafe + turn);
        double bRightPower = (forward + strafe - turn);

//        double fLeftPower = powerX + powerY + powerAngle;
//        double fRightPower = powerX - powerY - powerAngle;
//        double bLeftPower = powerX - powerY + powerAngle;
//        double bRightPower = powerX + powerY - powerAngle;

        double absMaxPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        if (absMaxPower > 1) {
            fLeftPower = fLeftPower / absMaxPower;
            fRightPower = fRightPower / absMaxPower;
            bLeftPower = bLeftPower / absMaxPower;
            bRightPower = bRightPower / absMaxPower;
        }
        double[] driveTrainPowers = {fLeftPower, fRightPower, bLeftPower, bRightPower};
        return driveTrainPowers;
    }

    public double[] calculatePower(Gamepad gamepad) {
        //negative because gamepad y is flip
        double forward = Math.cbrt(-gamepad.left_stick_y) * powerCoefficient;
        double turn = Math.cbrt(gamepad.right_stick_x) * powerCoefficient;
        double strafe = Math.cbrt(gamepad.left_stick_x) * powerCoefficient;

        KLog.d("drive", "forward " + forward);
        KLog.d("drive", "turn " + turn);
        KLog.d("drive", "strafe " + strafe);

        double fLeftPower = (forward + strafe + turn);
        double fRightPower = (forward - strafe - turn);
        double bLeftPower = (forward - strafe + turn);
        double bRightPower = (forward + strafe - turn);

        double absMaxPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        if (absMaxPower > 1) {
            fLeftPower = fLeftPower / absMaxPower;
            fRightPower = fRightPower / absMaxPower;
            bLeftPower = bLeftPower / absMaxPower;
            bRightPower = bRightPower / absMaxPower;
        }
        double[] driveTrainPowers = {fLeftPower, fRightPower, bLeftPower, bRightPower};
        return driveTrainPowers;
    }

    public double[] calculatePowerSmarter(Gamepad gamepad) {
        //negative because gamepad y is flip
        double forward = 1 * -gamepad.left_stick_y; //cube so fast is fast and slow is slow       * -gamepad.left_stick_y * -gamepad.left_stick_y
        double turn = 1 * gamepad.right_stick_x; //* gamepad.right_stick_x * gamepad.right_stick_x
        double strafe = 1 * gamepad.left_stick_x; //* gamepad.left_stick_x * gamepad.left_stick_x

        KLog.d("drive", "forward " + forward);
        KLog.d("drive", "turn " + turn);
        KLog.d("drive", "strafe " + strafe);

        final double turnReserve = 0.4;
        final double translationScale = 1.0 - turnReserve;

        double translationMagnitude = Math.max(Math.abs(forward) + Math.abs(strafe), 1.0);

        double scaledForward = (forward / translationMagnitude) * translationScale;
        double scaledStrafe = (strafe  / translationMagnitude) * translationScale;

        double scaledTurn = turn * turnReserve;

        KLog.d("drive", "scaled forward " + scaledForward);
        KLog.d("drive", "scaled strafe " + scaledStrafe);
        KLog.d("drive", "scaled turn " + scaledTurn);

        double fLeftPower = scaledForward + scaledStrafe + scaledTurn;
        double fRightPower = scaledForward - scaledStrafe - scaledTurn;
        double bLeftPower = scaledForward - scaledStrafe + scaledTurn;
        double bRightPower = scaledForward + scaledStrafe - scaledTurn;

        //safety
        double absMaxPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        if (absMaxPower > 1) {
            fLeftPower = fLeftPower / absMaxPower;
            fRightPower = fRightPower / absMaxPower;
            bLeftPower = bLeftPower / absMaxPower;
            bRightPower = bRightPower / absMaxPower;
        }

        double[] driveTrainPowers = {fLeftPower, fRightPower, bLeftPower, bRightPower};
        return driveTrainPowers;
    }

    public double[] calculatePowerFromVelocity(Gamepad gamepad, Position currentPosition) {
        //negative because gamepad y is flip
        double vxTarget = -gamepad.left_stick_y * MAX_X_VELOCITY_MM_S;
        double vThetaTarget = gamepad.right_stick_x * MAX_ANGLE_VELOCITY_RAD_S;
        double vyTarget = gamepad.left_stick_x * MAX_Y_VELOCITY_MM_S;

        double elapsedTime = System.currentTimeMillis() - startTimeMS;
        double dtSeconds  = (elapsedTime - lastMilli) / 1000.0; // ms to s
        if (dtSeconds <= 0) dtSeconds = 1e-3;

        double vxCurrent = (currentPosition.getX() - prevPosition.getX()) / dtSeconds;
        filteredVXCurrent = (FILTER_SMOOTHING_FACTOR * filteredVXCurrent) +
                    ((1 - FILTER_SMOOTHING_FACTOR) * vxCurrent);
        double vyCurrent = (currentPosition.getY() - prevPosition.getY()) / dtSeconds;
        filteredVYCurrent = (FILTER_SMOOTHING_FACTOR * filteredVYCurrent) +
                ((1 - FILTER_SMOOTHING_FACTOR) * vyCurrent);
        double dTheta = MathFunctions.angleWrapRad(currentPosition.getTheta() - prevPosition.getTheta());
        double vThetaCurrent  = dTheta / dtSeconds;
        filteredVThetaCurrent = (FILTER_SMOOTHING_FACTOR * filteredVThetaCurrent) +
                ((1 - FILTER_SMOOTHING_FACTOR) * vThetaCurrent);

        double vxCalc = vxTarget + K_px * (vxTarget - filteredVXCurrent);
            if (vxTarget == 0) vxCalc = 0;
        double vyCalc = vyTarget + K_py * (vyTarget - filteredVYCurrent);
            if (vyTarget == 0) vyCalc = 0;
        double vThetaCalc = vThetaTarget + K_pTheta * (vThetaTarget - filteredVThetaCurrent);
            if (vThetaTarget == 0) vThetaCalc = 0;

        //Convert the angular velocity into linear velocity
        double K_omega = (Odometry.getTrackWidthMm() / 2) + (WHEELBASE_LENGTH / 2); //mm/rad
        double omega = vThetaCalc * K_omega;

        double fLeftVelocity = vxCalc + vyCalc + omega;
        double bLeftVelocity = vxCalc - vyCalc + omega;
        double fRightVelocity = vxCalc - vyCalc - omega;
        double bRightVelocity = vxCalc + vyCalc - omega;

        double fLeftPower = fLeftVelocity / MAX_WHEEL_VELOCITY_MM_S;
        double bLeftPower = bLeftVelocity / MAX_WHEEL_VELOCITY_MM_S;
        double fRightPower = fRightVelocity / MAX_WHEEL_VELOCITY_MM_S;
        double bRightPower = bRightVelocity / MAX_WHEEL_VELOCITY_MM_S;

        double absMaxPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        if (absMaxPower > 1) {
            fLeftPower = fLeftPower / absMaxPower;
            fRightPower = fRightPower / absMaxPower;
            bLeftPower = bLeftPower / absMaxPower;
            bRightPower = bRightPower / absMaxPower;
        }

        double[] driveTrainPowers = {fLeftPower, fRightPower, bLeftPower, bRightPower};
        return driveTrainPowers;
    }

    public void move(Gamepad gamepad) {
        double[] driveTrainPower = calculatePower(gamepad);

        fLeft.setPower(driveTrainPower[0]);
        fRight.setPower(driveTrainPower[1]);
        bLeft.setPower(driveTrainPower[2]);
        bRight.setPower(driveTrainPower[3]);

        KLog.d("drive", "fLeft power " + driveTrainPower[0] + "fRight power " + driveTrainPower[1] + "bLeft power " + driveTrainPower[2] + "bRight power " + driveTrainPower[3]);
    }

    public void moveVelocity(Gamepad gamepad, Position currentPosition) {
        if (!hasStarted) {
            startTimeMS = System.currentTimeMillis();
            prevPosition = currentPosition;
            hasStarted = true;
        }

        double[] driveTrainPower = calculatePowerFromVelocity(gamepad, currentPosition);

        fLeft.setPower(driveTrainPower[0]);
        fRight.setPower(driveTrainPower[1]);
        bLeft.setPower(driveTrainPower[2]);
        bRight.setPower(driveTrainPower[3]);

        KLog.d("drive", "fLeft power " + driveTrainPower[0] + "fRight power " + driveTrainPower[1] + "bLeft power " + driveTrainPower[2] + "bRight power " + driveTrainPower[3]);
    }

    public void moveWithXYValues(double x, double y, Gamepad gamepad) {
        double[] driveTrainPower = calculatePower1(x, y, gamepad);

        fLeft.setPower(driveTrainPower[0]);
        fRight.setPower(driveTrainPower[1]);
        bLeft.setPower(driveTrainPower[2]);
        bRight.setPower(driveTrainPower[3]);

        KLog.d("drive", "fLeft power " + driveTrainPower[0]);
        KLog.d("drive", "fRight power " + driveTrainPower[1]);
        KLog.d("drive", "bLeft power " + driveTrainPower[2]);
        KLog.d("drive", "bRight power " + driveTrainPower[3]);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public double getPowerCoefficient() {
        return powerCoefficient;
    }

    public void setPowerCoefficient(double powerCoefficient) {
        this.powerCoefficient = powerCoefficient;
    }
}
