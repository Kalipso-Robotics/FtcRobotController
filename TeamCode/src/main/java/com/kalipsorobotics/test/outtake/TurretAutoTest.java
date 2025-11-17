package com.kalipsorobotics.test.outtake;

import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TurretAutoTest extends KTeleOp {

    DriveTrain driveTrain;
    IMUModule imuModule;

    Odometry odometry;

    Turret turret;
    TurretAutoAlign turretAutoAlign;
    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        DriveTrain.setInstanceNull();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        imuModule = IMUModule.getInstance(opModeUtilities);

        Odometry.setInstanceNull();
        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretConfig.RED_X_INIT_SETUP, TurretConfig.RED_Y_INIT_SETUP);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        KLog.d("Turret_Singleton", "Starting Auto");

        waitForStart();
        while(opModeIsActive()) {
            KLog.d("Turret_Singleton", "In Auto " + turret.turretMotor.getCurrentPosition());

            turretAutoAlign.updateCheckDone();
        }

        KLog.d("Turret_Singleton", "Before cleanup Auto " + turret.turretMotor.getCurrentPosition());
        cleanupRobot();
        KLog.d("Turret_Singleton", "After cleanup Auto " + turret.turretMotor.getCurrentPosition());

    }
}
