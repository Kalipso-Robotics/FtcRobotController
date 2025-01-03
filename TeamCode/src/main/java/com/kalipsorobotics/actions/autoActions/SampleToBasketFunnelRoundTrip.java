package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelReady;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.intoTheDeep.AutoBasket;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleToBasketFunnelRoundTrip extends KActionSet {

    public SampleToBasketFunnelRoundTrip(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, IntakeClaw intakeClaw, int sampleY){
        final int INTAKE_SAMPLE_X = -590-325;

        int outtakeXPos = -190;
        int outtakeYPos = 1020;

        PurePursuitAction moveToSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setName("moveToSample1");
        //bar to sample 1
        moveToSample1.addPoint(INTAKE_SAMPLE_X+465, sampleY, 180);
        moveToSample1.addPoint(INTAKE_SAMPLE_X+75, sampleY, 180);
        this.addAction(moveToSample1);

        IntakeFunnelReady intakeFunnelReady1 = new IntakeFunnelReady(intakeClaw, outtake);
        intakeFunnelReady1.setName("intakeFunnelReady1");
        this.addAction(intakeFunnelReady1);

        WaitAction waitBeforeFunnel1 = new WaitAction(250);
        waitBeforeFunnel1.setName("waitBeforeFunnel1");
        waitBeforeFunnel1.setDependentActions(moveToSample1, intakeFunnelReady1);
        this.addAction(waitBeforeFunnel1);

        PurePursuitAction funnelSample1 = new PurePursuitAction(driveTrain, wheelOdometry, 1.0/3200.0);
        funnelSample1.setName("funnelSample1");
        funnelSample1.setDependentActions(waitBeforeFunnel1);
        //bar to sample 1
        funnelSample1.addPoint(INTAKE_SAMPLE_X, sampleY, 180);
        this.addAction(funnelSample1);

        IntakeFunnelAction intakeFunnelAction1 = new IntakeFunnelAction(intakeClaw, outtake);
        intakeFunnelAction1.setName("intakeFunnelAction1");
        intakeFunnelAction1.setDependentActions(waitBeforeFunnel1);
        this.addAction(intakeFunnelAction1);

        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(intakeFunnelReady1, moveToSample1);
        this.addAction(outtakeTransferReady);

        IntakeTransferReady intakeTransferReady1 = new IntakeTransferReady(intakeClaw);
        intakeTransferReady1.setName("intakeTransferReady1");
        intakeTransferReady1.setDependentActions(intakeFunnelAction1, outtakeTransferReady);
        this.addAction(intakeTransferReady1);

        TransferAction transferAction1 = new TransferAction(intakeClaw, outtake);
        transferAction1.setName("transferAction1");
        transferAction1.setDependentActions(intakeTransferReady1, outtakeTransferReady);
        this.addAction(transferAction1);

        PurePursuitAction moveToBasket1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket1.setName("moveToBasket1");
        moveToBasket1.setDependentActions(intakeFunnelAction1, funnelSample1);
        //move sample 1 to basket
        moveToBasket1.addPoint(outtakeXPos, outtakeYPos, -135);
        this.addAction(moveToBasket1);

        BasketReadyAction basketReady1 = new BasketReadyAction(outtake);
        basketReady1.setName("basketReady1");
        basketReady1.setDependentActions(moveToBasket1, transferAction1);
        this.addAction(basketReady1);

        WaitAction waitAction1 = new WaitAction(100);
        waitAction1.setName("waitAction1");
        waitAction1.setDependentActions(basketReady1);
        this.addAction(waitAction1);

        KServoAutoAction openClaw1 = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw1.setName("openClaw1");
        openClaw1.setDependentActions(basketReady1);
        this.addAction(openClaw1);

        PurePursuitAction moveOutBasket1 = new PurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket1.setName("moveOutBasket1");
        moveOutBasket1.setDependentActions(openClaw1, waitAction1);
        moveOutBasket1.addPoint(outtakeXPos - 100, outtakeYPos - 100, -135);
        this.addAction(moveOutBasket1);
    }
}
//move robot to samples
//grab sample
//move to basket
//drop sample
//repeat
