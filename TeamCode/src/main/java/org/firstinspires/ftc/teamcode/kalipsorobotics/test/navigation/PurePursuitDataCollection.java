package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.GoBildaOdoModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Disabled
public class PurePursuitDataCollection extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);
        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);
        Odometry.setInstanceNull();
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain);
        purePursuitAction.addPoint(0, 0, 0);
        purePursuitAction.addPoint(600, 600, 0);
        purePursuitAction.addPoint(1200, -600, 90);
        purePursuitAction.addPoint(1800, -1200, 180);
        purePursuitAction.addPoint(1200, 1200, 0);
        purePursuitAction.addPoint(-600, 0, 90);
        purePursuitAction.addPoint(1800, -600, 180);
        purePursuitAction.addPoint(0, 0, 0);

        PurePursuitFileWriter purePursuitFileWriter = new PurePursuitFileWriter("datafordabigduong", opModeUtilities);

        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);

        waitForStart();
        while (opModeIsActive()) {
            //purePursuitAction.updateCheckDone();
            purePursuitFileWriter.writePurePursuitData(SharedData.getOdometryPositionMap(), driveTrain);
            driveAction.move(gamepad1);
        }
        purePursuitFileWriter.close();

        OpModeUtilities.shutdownExecutorService(executorService);

    }
}
