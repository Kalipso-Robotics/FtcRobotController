//package com.kalipsorobotics.decode.calibrations;
//
//import static com.kalipsorobotics.actions.shooter.PushBall.STATIC_LEFT_POS;
//import static com.kalipsorobotics.actions.shooter.PushBall.STATIC_RIGHT_POS;
//import static com.kalipsorobotics.modules.Revolver.REVOLVER_INDEX_0;
//
//import com.kalipsorobotics.modules.Revolver;
//import com.kalipsorobotics.modules.shooter.Shooter;
//import com.kalipsorobotics.utilities.KLog;
//import com.kalipsorobotics.utilities.KTeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp
//public class RevolverCalibration extends KTeleOp {
//
//    private static final double POSITION_INCREMENT = 0.01;
//
//    Shooter shooter = null;
//    Revolver revolver = null;
//
//
//    @Override
//    protected void initializeRobot() {
//        super.initializeRobot();
//        shooter = new Shooter(opModeUtilities);
//        revolver = new Revolver(opModeUtilities);
//
//    }
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        double revolverPosition = REVOLVER_INDEX_0;
//
//        double kickerRightPosition = STATIC_RIGHT_POS;
//        double kickerLeftPosition = STATIC_LEFT_POS;
//
//        initializeRobot();
//        waitForStart();
//        while (opModeIsActive()) {
//            if (kGamePad2.isDpadLeftFirstPressed()) {
//                revolverPosition += POSITION_INCREMENT;
//                revolver.getRevolverServo().setPosition(revolverPosition);
//            } else if (kGamePad2.isDpadRightFirstPressed()) {
//                revolverPosition -= POSITION_INCREMENT;
//                revolver.getRevolverServo().setPosition(revolverPosition);
//            }
//
//            if (kGamePad2.isDpadUpFirstPressed()) {
//                kickerRightPosition -= POSITION_INCREMENT;
//                shooter.getKickerRight().setPosition(kickerRightPosition);
//            } else if (kGamePad2.isDpadDownFirstPressed()) {
//                kickerRightPosition += POSITION_INCREMENT;
//                shooter.getKickerRight().setPosition(kickerRightPosition);
//            }
//
//            if (kGamePad2.isButtonYFirstPressed()) {
//                kickerLeftPosition += POSITION_INCREMENT;
//                shooter.getKickerLeft().setPosition(kickerLeftPosition);
//            } else if (kGamePad2.isButtonAFirstPressed()) {
//                kickerLeftPosition -= POSITION_INCREMENT;
//                shooter.getKickerLeft().setPosition(kickerLeftPosition);
//            }
//
//
//
//            KLog.d("revolver_position", "revolverPosition" + revolverPosition);
//            KLog.d("kicker_right_position", "kickerRightPosition" + kickerRightPosition);
//            KLog.d("kicker_left_position", "kickerLeftPosition" + kickerLeftPosition);
//
//
//            telemetry.addData("revolver_position", revolverPosition); //0.11 0.51 0.91
//            telemetry.addData("kicker_right_position", kickerRightPosition);
//            telemetry.addData("kicker_left_position", kickerLeftPosition);
//            telemetry.update();
//        }
//
//        cleanupRobot();
//
//    }
//}
