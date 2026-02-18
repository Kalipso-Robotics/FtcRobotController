package org.firstinspires.ftc.teamcode.kalipsorobotics.test.outtake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

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
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

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
