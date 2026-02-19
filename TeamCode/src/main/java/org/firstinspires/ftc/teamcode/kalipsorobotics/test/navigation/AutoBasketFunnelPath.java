package org.firstinspires.ftc.teamcode.kalipsorobotics.test.navigation;


import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.GoBildaOdoModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class AutoBasketFunnelPath extends LinearOpMode {

    protected boolean isFirstMoveSpecimen = true;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoBasket = new KActionSet();

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        GoBildaOdoModule.setInstanceNull();
        GoBildaOdoModule goBildaOdoModule = new GoBildaOdoModule(opModeUtilities);

        Odometry.setInstanceNull();
        Odometry wheelOdometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        SharedData.resetOdometryWheelIMUPosition();

        final int INTAKE_SAMPLE_X = -590-300;

        final int OUTTAKE_X_POS = -155; //-140
        final int OUTTAKE_Y_POS = 980;

        final int INTAKE_SAMPLE_X_FUNNEL = -590-325;




//        int outtakeXPos = -190;
//        int outtakeYPos = 1020;

        //================begin of first specimen====================
        AdaptivePurePursuitAction moveOutSpecimen = null;
        AdaptivePurePursuitAction moveToBasket1 = null;
        AdaptivePurePursuitAction moveOutBasket1 = null;

        moveToBasket1 = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket1.setName("moveToBasket3");
        //move sample 3 to basket
        moveToBasket1.addPoint(OUTTAKE_X_POS - 100, OUTTAKE_Y_POS - 125, -135);
        moveToBasket1.addPoint(OUTTAKE_X_POS, OUTTAKE_Y_POS, -135);
        redAutoBasket.addAction(moveToBasket1);

        moveOutBasket1 = new AdaptivePurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket1.setName("moveOutBasket3");
        moveOutBasket1.setDependentActions(moveToBasket1);
        moveOutBasket1.addPoint(OUTTAKE_X_POS - 150, OUTTAKE_Y_POS - 150, -135);
        redAutoBasket.addAction(moveOutBasket1);
        //=================end of first specimen=================


        //================begin of first basket====================
        AdaptivePurePursuitAction moveToSample1 = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setName("moveToSample1");
        //bar to sample 1
        moveToSample1.addPoint(INTAKE_SAMPLE_X_FUNNEL + 415, 850, 180);
        moveToSample1.addPoint(INTAKE_SAMPLE_X_FUNNEL + 75, 850, 180);
        moveToSample1.addPoint(INTAKE_SAMPLE_X_FUNNEL - 250, 850, 180);
        redAutoBasket.addAction(moveToSample1);

        AdaptivePurePursuitAction moveToBasket2 = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket2.setName("moveToBasket2");
        moveToBasket2.setDependentActions(moveToSample1); // Depends on transfer so intake slides dont move
        // while
        // moving
        //move sample 1 to basket
        moveToBasket2.addPoint(INTAKE_SAMPLE_X_FUNNEL - 300, OUTTAKE_Y_POS - 225,-180);
        moveToBasket2.addPoint(OUTTAKE_X_POS - 100, OUTTAKE_Y_POS - 125, -135);
        moveToBasket2.addPoint(OUTTAKE_X_POS, OUTTAKE_Y_POS, -135);
        redAutoBasket.addAction(moveToBasket2);

        AdaptivePurePursuitAction moveOutBasket2 = new AdaptivePurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket2.setName("moveOutBasket2");
        moveOutBasket2.setDependentActions(moveToBasket2);
        moveOutBasket2.addPoint(OUTTAKE_X_POS - 150, OUTTAKE_Y_POS - 150, -135);
        redAutoBasket.addAction(moveOutBasket2);
        //===============end of first basket===============


        //===============start of second basket===============
        AdaptivePurePursuitAction moveToSample2 = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        moveToSample2.setName("moveToSample2");
        //bar to sample 1
        moveToSample2.addPoint(INTAKE_SAMPLE_X_FUNNEL + 415, 1100, 180);
        moveToSample2.addPoint(INTAKE_SAMPLE_X_FUNNEL + 75, 1100, 180);
        moveToSample2.addPoint(INTAKE_SAMPLE_X_FUNNEL - 250 - 75, 1100, 180);
        redAutoBasket.addAction(moveToSample2);

        AdaptivePurePursuitAction moveToBasket3 = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket3.setName("moveToBasket3");
        moveToBasket3.setDependentActions(moveToSample2); // Depends on transfer so intake slides dont move
        // while
        // moving
        //move sample 1 to basket
        moveToBasket3.addPoint(INTAKE_SAMPLE_X_FUNNEL - 300, OUTTAKE_Y_POS - 225,-180);
        moveToBasket3.addPoint(OUTTAKE_X_POS - 100, OUTTAKE_Y_POS - 125, -135);
        moveToBasket3.addPoint(OUTTAKE_X_POS, OUTTAKE_Y_POS, -135);
        redAutoBasket.addAction(moveToBasket3);

        AdaptivePurePursuitAction moveOutBasket3 = new AdaptivePurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket3.setName("moveOutBasket3");
        moveOutBasket3.setDependentActions(moveToBasket3);
        moveOutBasket3.addPoint(OUTTAKE_X_POS - 150, OUTTAKE_Y_POS - 150, -135);
        redAutoBasket.addAction(moveOutBasket3);
        //===============end of second basket===============


        //===============start of third basket===============

        AdaptivePurePursuitAction moveToSample3 = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        moveToSample3.setName("moveToSample3");
        moveToSample3.setDependentActions(moveOutBasket3);
        moveToSample3.addPoint(INTAKE_SAMPLE_X - 40, 765, 90);
        redAutoBasket.addAction(moveToSample3);

        AdaptivePurePursuitAction moveToBasket4 = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket4.setName("moveToBasket4");
        moveToBasket4.setDependentActions(moveToSample3);
        //move sample 3 to basket
        moveToBasket4.addPoint(OUTTAKE_X_POS - 250, OUTTAKE_Y_POS - 125, -135);
        moveToBasket4.addPoint(OUTTAKE_X_POS, OUTTAKE_Y_POS + 50, -135);
        redAutoBasket.addAction(moveToBasket4);

        AdaptivePurePursuitAction moveOutBasket4 = new AdaptivePurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket4.setName("moveOutBasket4");
        moveOutBasket4.setDependentActions(moveToBasket4);
        moveOutBasket4.addPoint(OUTTAKE_X_POS - 150,OUTTAKE_Y_POS - 150, -135);
        redAutoBasket.addAction(moveOutBasket4);

        AdaptivePurePursuitAction park = new AdaptivePurePursuitAction(driveTrain, wheelOdometry);
        park.setName("park");
        park.setDependentActions(moveOutBasket3);
        park.addPoint(-1225, 700, 45); // 610
        park.addPoint(-1325, 190, 90);
        redAutoBasket.addAction(park);

        telemetry.addLine("init finished");
        telemetry.update();

        ExecutorService executorService = Executors.newSingleThreadExecutor();

        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);

        while (opModeIsActive()) {

            redAutoBasket.updateCheckDone();
            KLog.d("homePos", () -> SharedData.getOdometryWheelIMUPosition().toString());
            KLog.d("homePosMap", () -> SharedData.getOdometryPositionMap().toString());

        }

        OpModeUtilities.shutdownExecutorService(executorService);
        //KLog.d("executor service",
        //        "after shutdown" + SharedData.getOdometryPosition() + "is shutdown " + executorService.isShutdown()
        //        + "is terminated " + executorService.isTerminated());

    }
}