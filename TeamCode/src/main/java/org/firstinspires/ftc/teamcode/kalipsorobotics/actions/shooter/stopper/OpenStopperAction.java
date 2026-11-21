package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Opens the stopper and is done when the servo has had time to get there.
 * Auto: {@code new OpenStopperAction(stopper)}
 * Pattern source: CloseStopperAction (its mirror).
 */
public class OpenStopperAction extends KActionSet {
    public static final String TAG = "Shooting_Stopper";

    public OpenStopperAction(Stopper stopper) {
        KLog.d(TAG, () -> "event=start open");
        KServoAutoAction openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
        openStopper.setName("openStopper");
        this.addAction(openStopper);
    }
}
