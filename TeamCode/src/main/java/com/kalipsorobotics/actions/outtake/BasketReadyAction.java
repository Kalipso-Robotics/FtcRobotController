package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Outtake;

public class BasketReadyAction extends KActionSet {
    public BasketReadyAction(Outtake outtake) {
        KServoAutoAction outtakeClawActionClose = new KServoAutoAction(outtake.getOuttakeClawServo(),
                OuttakeClawAction.OUTTAKE_CLAW_CLOSE_POS);
        outtakeClawActionClose.setName("outtakeClawActionClose");
        this.addAction(outtakeClawActionClose);

        MoveOuttakeLSAction raiseSlidesBasket = new MoveOuttakeLSAction(outtake, 800);
        raiseSlidesBasket.setName("raiseSlidesBasket");
        this.addAction(raiseSlidesBasket);

        KServoAutoAction outtakePivotActionUp = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_HALF_POS);
        outtakePivotActionUp.setName("outtakePivotActionUp");
        this.addAction(outtakePivotActionUp);
    }
}
