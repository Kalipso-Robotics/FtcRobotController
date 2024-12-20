package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class BlueAutoBasket extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);

        PurePursuitAction moveToSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setDependentActions(moveToSpecimenBar);

        PurePursuitAction moveToBasket1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket1.setDependentActions(moveToSample1);

        PurePursuitAction moveToSample2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample2.setDependentActions(moveToBasket1);

        PurePursuitAction moveToBasket2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket2.setDependentActions(moveToSample2);

        PurePursuitAction moveToSample3 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample3.setDependentActions(moveToBasket2);

        PurePursuitAction moveToBasket3 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket3.setDependentActions(moveToSample3);

        moveToSpecimenBar.setSleep(1000);

        int outtakeXPos = -350;
        int outtakeYPos = 900;

        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-740, -300, 0);

        //bar to sample 1
        moveToSample1.addPoint(-620, 820, 180);

        //intake

        //move sample to basket sample 1
        moveToBasket1.addPoint(outtakeXPos, outtakeYPos, -135);

        //move basket to sample 2
        moveToSample2.addPoint(-620, 950, 180);

        //intake

        //move sample to basket sample 2
        moveToBasket2.addPoint(outtakeXPos, outtakeYPos, -135);

        //move basket to third sample
        moveToSample3.addPoint(-930, 1010, 90);

        //move sample to basket 3
        moveToBasket3.addPoint(outtakeXPos, outtakeYPos, -135);

        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            moveToSpecimenBar.update();

            moveToSample1.updateCheckDone();
            moveToBasket1.updateCheckDone();
            moveToSample2.updateCheckDone();
            moveToBasket2.updateCheckDone();
            moveToSample3.updateCheckDone();
            moveToBasket3.updateCheckDone();

        }

    }
}
