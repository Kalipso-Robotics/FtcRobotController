package com.kalipsorobotics.test.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.actionUtilities.turretActions.TurretAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Odometry Test For New Robot")
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
        TurretAction turretAction = new TurretAction(odometry, turret);

        waitForStart();
        while(opModeIsActive()) {
            Log.d("OdometryTest","x: " + odometry.update().getX() + "y: " + odometry.update().getY() + "theta: " + odometry.getIMUHeading());
            turretAction.updateCheckDone();
        }
    }
}
