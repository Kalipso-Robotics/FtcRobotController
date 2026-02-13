package com.kalipsorobotics.test.outtake;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KOpMode;
import com.kalipsorobotics.utilities.OpModeUtilities;

//@Autonomous
public class TurretAutoTest extends KOpMode {

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
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

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
