package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;

public class CloseStopperAction extends KActionSet {
    public CloseStopperAction(Stopper stopper) {
        KServoAutoAction closeStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_CLOSED_POS);

        closeStopper.setName("closeStopper");
        this.addAction(closeStopper);
    }
}
