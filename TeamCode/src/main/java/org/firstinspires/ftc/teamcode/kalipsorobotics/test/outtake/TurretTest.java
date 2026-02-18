package org.firstinspires.ftc.teamcode.kalipsorobotics.test.outtake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

//@TeleOp
public class TurretTest extends KOpMode {

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
        OpModeUtilities.runOdometryExecutorService(odoExecutorService, odometry);

        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, allianceColor);

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
