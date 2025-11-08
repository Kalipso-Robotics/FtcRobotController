package com.kalipsorobotics.decode.calibrations;

import static com.kalipsorobotics.modules.Revolver.REVOLVER_INDEX_0;

import com.kalipsorobotics.modules.Pusher;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RevolverCalibration extends KTeleOp {

    private static final double POSITION_INCREMENT = 0.01;

    Pusher pusher = null;


    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        pusher = new Pusher(opModeUtilities);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        double stopperPosition = 1;


        initializeRobot();
        waitForStart();
        while (opModeIsActive()) {
            if (kGamePad2.isDpadLeftFirstPressed()) {
                stopperPosition += POSITION_INCREMENT;
                pusher.getStopper().setPosition(stopperPosition);
            } else if (kGamePad2.isDpadRightFirstPressed()) {
                stopperPosition -= POSITION_INCREMENT;
                pusher.getStopper().setPosition(stopperPosition);
            }


            KLog.d("stopperPosition", "stopperPosition" + stopperPosition);
            telemetry.addData("stopperPosition", stopperPosition); //0.11 0.51 0.91
            telemetry.update();
        }

        cleanupRobot();

    }
}
