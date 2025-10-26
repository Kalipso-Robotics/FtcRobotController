//package com.kalipsorobotics.test;
//
//import com.kalipsorobotics.modules.MotifColor;
//import com.kalipsorobotics.utilities.KColor;
//import com.kalipsorobotics.utilities.KLog;
//import com.kalipsorobotics.utilities.OpModeUtilities;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp
//public class ColorTest extends LinearOpMode {
//
//
//    MotifColor currentColor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, this.telemetry);
//
//        RevColorSensorV3 revColor = hardwareMap.get(RevColorSensorV3.class, "revColor1");
//        RevColorSensorV3 revColor2 = hardwareMap.get(RevColorSensorV3.class, "revColor2");
//        RevColorSensorV3 revColor3 = hardwareMap.get(RevColorSensorV3.class, "revColor3");
//
//        waitForStart();
//        while(opModeIsActive()) {
////            KLog.d("revColor", "rev1" + ColorCalibrationDetection.detectColor("revColor1", revColor, opModeUtilities) + " " + KColor.printHSV(revColor) + " rev2 " + ColorCalibrationDetection.detectColor("revColor2", revColor2, opModeUtilities) + " " + KColor.printHSV(revColor2) + " rev3 " + ColorCalibrationDetection.detectColor("revColor3", revColor3, opModeUtilities) + " " +  KColor.printHSV(revColor3));
//            KLog.d("revColor", "rev1" + " " + KColor.printHSV(revColor) + " rev2 " +  " " + KColor.printHSV(revColor2) + " rev3 " + " " +  KColor.printHSV(revColor3));
//            telemetry.addLine("revColor1: " + KColorDetection.detectColor("revColor1", revColor, opModeUtilities));
////            telemetry.addLine(KColor.printColor(revColor));
//            telemetry.addLine(KColor.printHSV(revColor));
//            telemetry.addLine("revColor2: " + KColorDetection.detectColor("revColor2", revColor2, opModeUtilities));
////            telemetry.addLine(KColor.printColor(revColor2));
//            telemetry.addLine(KColor.printHSV(revColor2));
//            telemetry.addLine("revColor3: " + KColorDetection.detectColor("revColor3", revColor3, opModeUtilities));
////            telemetry.addLine(KColor.printColor(revColor3));
//            telemetry.addLine(KColor.printHSV(revColor3));
//            telemetry.update();
//        }
//    }
//}


//darren cant digest cheese