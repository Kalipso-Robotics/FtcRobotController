package com.kalipsorobotics.test.outtake;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TurretTest", group = "Linear OpMode")
public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        Turret.setInstanceNull();
        Turret turret = Turret.getInstance(opModeUtilities);
        TurretAutoAlign turretAutoAlign = new TurretAutoAlign(turret);

        waitForStart();
        while(opModeIsActive()) {
            KLog.d("OdometryTest","x: " + odometry.update().getX() + "y: " + odometry.update().getY() + "theta: " + odometry.getIMUHeading());
            turretAutoAlign.updateCheckDone();
        }
    }
}
