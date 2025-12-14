package com.kalipsorobotics.actions.shooter.stopper;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.Stopper;

public class CloseStopperAction extends KActionSet {
    public CloseStopperAction(Stopper stopper) {
        KServoAutoAction closeStopper = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);

        closeStopper.setName("closeStopper");
        this.addAction(closeStopper);
    }
}
