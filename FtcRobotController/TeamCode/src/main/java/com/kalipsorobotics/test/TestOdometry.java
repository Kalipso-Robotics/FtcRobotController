//package com.kalipsorobotics.test;
//
//import com.kalipsorobotics.localization.Odometry;
//import com.kalipsorobotics.localization.RobotMovement;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import com.kalipsorobotics.utilities.OpModeUtilities;
//import com.kalipsorobotics.modules.DriveTrain;
//
//
//@TeleOp(name = "Fresh Odometry")
//public class TestOdometry extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
//        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
//        Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
//        RobotMovement robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);
//
//        waitForStart();
//        while (opModeUtilities.getOpMode().opModeIsActive()) {
//            odometry.updatePosition();
//        }
//    }
//}
