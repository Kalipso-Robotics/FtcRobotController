package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

import org.checkerframework.checker.units.qual.K;

public class IntakeTransferReady extends KActionSet {

    public IntakeTransferReady(IntakeClaw intake) {
        this(intake, false);
    }

    public IntakeTransferReady(IntakeClaw intake, boolean isExtendLinkageToMid) {

        KServoAutoAction unlockRatchet = null;
        KServoAutoAction linkageToMid = null;

        if (isExtendLinkageToMid) {
            unlockRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(),
                    IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);
            unlockRatchet.setName("unlockRatchet");
            this.addAction(unlockRatchet);

            linkageToMid = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_SAMPLE_TRANSFER_READY_HALF_POS);
            linkageToMid.setName("linkageToMid");
            this.addAction(linkageToMid);

        }

        KServoAutoAction lockRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(), IntakeClaw.INTAKE_RATCHET_LOCK_POS);
        lockRatchet.setName("lockRatchet");
        this.addAction(lockRatchet);
        lockRatchet.setDependentActions(linkageToMid, unlockRatchet);


        KServoAutoAction moveBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_TRANSFER_READY_POS);
        moveBigSweep.setName("moveBigSweep");
        this.addAction(moveBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        this.addAction(moveSmallSweep);

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        //moveBigPivot.setDependentActions(linkageToMid);
        this.addAction(moveBigPivot);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        //moveSmallPivot.setDependentActions(linkageToMid);
        this.addAction(moveSmallPivot);

        KServoAutoAction linkageRetract = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkageRetract.setName("linkageRetract");
        linkageRetract.setDependentActions(linkageToMid, moveBigPivot,moveSmallPivot);
        this.addAction(linkageRetract);

        WaitAction wait = new WaitAction(100);
        wait.setName("wait");
        wait.setDependentActions(moveSmallPivot, moveBigPivot, moveSmallSweep, moveBigSweep, linkageRetract);
        this.addAction(wait);

    }
}

//linkage retract
//big sweep parallel to robot (same value as intake ready)
// small sweep perpendicular to robot pos (same value as intake ready)
//big pivot goes to transfer ready pos
//small pivot goes to transfer ready pos
