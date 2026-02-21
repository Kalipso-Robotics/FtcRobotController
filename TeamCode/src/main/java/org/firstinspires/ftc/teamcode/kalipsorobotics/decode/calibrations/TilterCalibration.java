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
        double tilterPositionRight = 0.5;

        double tilterPositionLeft = 0.5;


        initializeRobot();
        waitForStart();
        while (opModeIsActive()) {

            if (kGamePad2.isDpadLeftFirstPressed()) {
                tilterPositionRight += POSITION_INCREMENT;
                tilter.getTilterRight().setPosition(tilterPositionRight);
            } else if (kGamePad2.isDpadRightFirstPressed()) {
                tilterPositionRight -= POSITION_INCREMENT;
                tilter.getTilterRight().setPosition(tilterPositionRight);
            }

            if (kGamePad2.isDpadUpFirstPressed()) {
                tilterPositionLeft += POSITION_INCREMENT;
                tilter.getTilterLeft().setPosition(tilterPositionLeft);
            } else if (kGamePad2.isDpadDownFirstPressed()) {
                tilterPositionLeft -= POSITION_INCREMENT;
                tilter.getTilterLeft().setPosition(tilterPositionLeft);
            }



            KLog.d("tilterPositionRight", "tilterPositionRight" + tilterPositionRight);
            telemetry.addData("tilterPositionRight", tilterPositionRight); //0.11 0.51 0.91
            telemetry.addData("tilterPositionLeft", tilterPositionLeft); //0.11 0.51 0.91
            telemetry.update();
        }

        cleanupRobot();

    }
}
