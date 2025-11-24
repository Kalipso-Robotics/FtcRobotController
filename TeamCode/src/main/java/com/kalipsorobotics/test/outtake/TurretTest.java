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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TurretTest extends KTeleOp {

    DriveTrain driveTrain;
    IMUModule imuModule;

    Odometry odometry;

    Turret turret;
    TurretAutoAlign turretAutoAlign;
    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        driveTrain = DriveTrain.getInstance(opModeUtilities);
        imuModule = IMUModule.getInstance(opModeUtilities);

        odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretConfig.X_INIT_SETUP, TurretConfig.Y_INIT_SETUP);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        KLog.d("Turret_Singleton", "Starting TeleOp " + turret.turretMotor.getCurrentPosition());

        waitForStart();
        while(opModeIsActive()) {
            KLog.d("Turret_Singleton", "In TeleOp " + turret.turretMotor.getCurrentPosition());

            turretAutoAlign.updateCheckDone();
        }
        cleanupRobot();
    }
}
