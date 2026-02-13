//package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;
//
//import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
//import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
//import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.GoBildaOdoModule;
//import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
//import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
//import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;
//
//@Autonomous(name="TestPurePursuitPath")
//public class TestPurePursuitPath extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
//
//        DriveTrain.setInstanceNull();
//        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
//
//        IMUModule.setInstanceNull();
//        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
//
//        GoBildaOdoModule.setInstanceNull();
//        GoBildaOdoModule goBildaOdoModule = GoBildaOdoModule.getInstance(opModeUtilities);
//
//        Odometry.setInstanceNull();
//        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, goBildaOdoModule);
//
//
//        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain);
////        purePursuitAction.addPoint(600, 0, 0);
////        purePursuitAction.addPoint(600, 600, 90);
////        purePursuitAction.addPoint(1200, 600, 180);
////        purePursuitAction.addPoint(1200, 0, -90);
////        purePursuitAction.addPoint(0, 0, -90);
//        purePursuitAction.addPoint(0, 0, 0);
//        purePursuitAction.addPoint(1300, 300, 0);
//        purePursuitAction.addPoint(350, 300, 0);
//        purePursuitAction.addPoint(1300, 300, 0);
//        purePursuitAction.addPoint(1300, 600, 0);
//        purePursuitAction.addPoint(350, 600, 0);
//        purePursuitAction.addPoint(1300, 600, 0);
//        purePursuitAction.addPoint(1300, 800, 0);
//        purePursuitAction.addPoint(350, 800, 0);
//        purePursuitAction.addPoint(1200, 1200, 90);
//        purePursuitAction.addPoint(1200, 0, 45);
//        purePursuitAction.addPoint(600, 0, 0);
//
//
//
//        ExecutorService executorService = Executors.newSingleThreadExecutor();
//        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
//
//        waitForStart();
//        while (opModeIsActive()) {
//            purePursuitAction.updateCheckDone();
//
//            if (purePursuitAction.getIsDone()) {
//                break;
//            }
//        }
//
//        OpModeUtilities.shutdownExecutorService(executorService);
//    }
//}