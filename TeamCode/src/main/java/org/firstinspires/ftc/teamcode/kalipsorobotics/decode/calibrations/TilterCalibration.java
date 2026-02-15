package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.calibrations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Tilter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;


@TeleOp
public class TilterCalibration extends KOpMode {


    private static final double POSITION_INCREMENT = 0.01;

    Tilter tilter = null;



    @Override
    protected void initializeRobot() {
        super.initializeRobot();

        tilter = new Tilter(opModeUtilities);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        double tilterPosition = 0.5;


        initializeRobot();
        waitForStart();
        while (opModeIsActive()) {

            if (kGamePad2.isDpadLeftFirstPressed()) {
                tilterPosition += POSITION_INCREMENT;
                tilter.getTilter().setPosition(tilterPosition);
            } else if (kGamePad2.isDpadRightFirstPressed()) {
                tilterPosition -= POSITION_INCREMENT;
                tilter.getTilter().setPosition(tilterPosition);
            }



            KLog.d("tilterPosition", "tilterPosition" + tilterPosition);
            telemetry.addData("tilterPosition", tilterPosition); //0.11 0.51 0.91
            telemetry.update();
        }

        cleanupRobot();

    }
}
