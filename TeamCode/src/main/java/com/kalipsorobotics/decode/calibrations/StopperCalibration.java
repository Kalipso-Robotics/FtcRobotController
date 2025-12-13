package com.kalipsorobotics.decode.calibrations;

import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
public class StopperCalibration extends KOpMode {

    private static final double POSITION_INCREMENT = 0.01;

    Stopper stopper = null;


    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        stopper = new Stopper(opModeUtilities);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        double stopperPosition = 1;


        initializeRobot();
        waitForStart();
        while (opModeIsActive()) {
            if (kGamePad2.isDpadLeftFirstPressed()) {
                stopperPosition += POSITION_INCREMENT;
                stopper.getStopper().setPosition(stopperPosition);
            } else if (kGamePad2.isDpadRightFirstPressed()) {
                stopperPosition -= POSITION_INCREMENT;
                stopper.getStopper().setPosition(stopperPosition);
            }


            KLog.d("stopperPosition", "stopperPosition" + stopperPosition);
            telemetry.addData("stopperPosition", stopperPosition); //0.11 0.51 0.91
            telemetry.update();
        }

        cleanupRobot();

    }
}
