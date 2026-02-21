package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Tilter;

public class TiltUp extends KActionSet {

    public TiltUp(Tilter tilter) {
        KServoAutoAction tiltLeftUp = new KServoAutoAction(tilter.getTilterLeft(), ModuleConfig.TILT_LEFT_UP_POS);
        tiltLeftUp.setName("tiltLeftUp");
        this.addAction(tiltLeftUp);

        KServoAutoAction tiltRightUp = new KServoAutoAction(tilter.getTilterRight(), ModuleConfig.TILT_RIGHT_UP_POS);
        tiltRightUp.setName("tiltRightUp");
        this.addAction(tiltRightUp);
    }

}
