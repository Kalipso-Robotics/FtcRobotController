package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Tilter;

public class TiltDown extends KActionSet {

    public TiltDown(Tilter tilter) {
        KServoAutoAction tiltLeftUp = new KServoAutoAction(tilter.getTilterLeft(), ModuleConfig.TILT_LEFT_DOWN_POS);
        tiltLeftUp.setName("tiltLeftUp");
        this.addAction(tiltLeftUp);

        KServoAutoAction tiltRightUp = new KServoAutoAction(tilter.getTilterRight(), ModuleConfig.TILT_RIGHT_DOWN_POS);
        tiltRightUp.setName("tiltRightUp");
        this.addAction(tiltRightUp);
    }

}
