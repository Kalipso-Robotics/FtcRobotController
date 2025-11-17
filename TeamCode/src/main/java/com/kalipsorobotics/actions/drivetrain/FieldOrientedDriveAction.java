package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldOrientedDriveAction {
    OpModeUtilities opModeUtilities;

    DriveTrain driveTrain;
    IMUModule imuModule;
    private final DcMotor fLeft, fRight, bLeft, bRight;
    private final DcMotor backEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor leftEncoder;

    private final double[] driveTrainPower = new double[4];

    public FieldOrientedDriveAction(DriveTrain driveTrain, IMUModule imuModule) {
        this.driveTrain = driveTrain;
        this.imuModule = imuModule;
        this.fLeft = driveTrain.getfLeft();
        this.fRight = driveTrain.getfRight();
        this.bLeft = driveTrain.getbLeft();
        this.bRight = driveTrain.getbRight();
        this.rightEncoder = driveTrain.getRightEncoder();
        this.leftEncoder = driveTrain.getLeftEncoder();
        this.backEncoder = driveTrain.getBackEncoder();
    }

    public double[] calculateFieldOrientedPower(double gamepadx, double gamepady, Gamepad gamepad) {
        double forward = -gamepady * -gamepady * -gamepady;
        double turn = gamepad.right_stick_x * gamepad.right_stick_x * gamepad.right_stick_x;
        double strafe = gamepadx * gamepadx * gamepadx;

        double botHeading = imuModule.getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        KLog.d("field_drive", "forward " + forward);
        KLog.d("field_drive", "turn " + turn);
        KLog.d("field_drive", "strafe " + strafe);
        KLog.d("field_drive", "botHeading " + Math.toDegrees(botHeading));
        KLog.d("field_drive", "rotX " + rotX);
        KLog.d("field_drive", "rotY " + rotY);

        double fLeftPower = (rotY + rotX + turn);
        double fRightPower = (rotY - rotX - turn);
        double bLeftPower = (rotY - rotX + turn);
        double bRightPower = (rotY + rotX - turn);

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

    public double[] calculateFieldOrientedPowerFromGamepad(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y * -gamepad.left_stick_y * -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x * gamepad.right_stick_x * gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x * gamepad.left_stick_x * gamepad.left_stick_x;

        double botHeading = imuModule.getIMU().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        KLog.d("field_drive", "forward " + forward);
        KLog.d("field_drive", "turn " + turn);
        KLog.d("field_drive", "strafe " + strafe);
        KLog.d("field_drive", "botHeading " + Math.toDegrees(botHeading));
        KLog.d("field_drive", "rotX " + rotX);
        KLog.d("field_drive", "rotY " + rotY);

        double fLeftPower = (rotY + rotX + turn);
        double fRightPower = (rotY - rotX - turn);
        double bLeftPower = (rotY - rotX + turn);
        double bRightPower = (rotY + rotX - turn);

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

    public void moveFieldOriented(Gamepad gamepad) {
        double[] driveTrainPower = calculateFieldOrientedPowerFromGamepad(gamepad);

        fLeft.setPower(driveTrainPower[0]);
        fRight.setPower(driveTrainPower[1]);
        bLeft.setPower(driveTrainPower[2]);
        bRight.setPower(driveTrainPower[3]);

        KLog.d("field_drive", "fLeft power " + driveTrainPower[0]);
        KLog.d("field_drive", "fRight power " + driveTrainPower[1]);
        KLog.d("field_drive", "bLeft power " + driveTrainPower[2]);
        KLog.d("field_drive", "bRight power " + driveTrainPower[3]);
    }

    public void moveFieldOrientedWithXYValues(double x, double y, Gamepad gamepad) {
        double[] driveTrainPower = calculateFieldOrientedPower(x, y, gamepad);

        fLeft.setPower(driveTrainPower[0]);
        fRight.setPower(driveTrainPower[1]);
        bLeft.setPower(driveTrainPower[2]);
        bRight.setPower(driveTrainPower[3]);

        KLog.d("field_drive", "fLeft power " + driveTrainPower[0]);
        KLog.d("field_drive", "fRight power " + driveTrainPower[1]);
        KLog.d("field_drive", "bLeft power " + driveTrainPower[2]);
        KLog.d("field_drive", "bRight power " + driveTrainPower[3]);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
