package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Tilter;

public class TiltDown extends KActionSet {

    public TiltDown(Tilter tilter) {
        KServoAutoAction tiltLeftDown = new KServoAutoAction(tilter.getTilterLeft(), ModuleConfig.TILT_LEFT_DOWN_POS);
        tiltLeftDown.setName("tiltLeftDown");
        this.addAction(tiltLeftDown);

        KServoAutoAction tiltRightDown = new KServoAutoAction(tilter.getTilterRight(), ModuleConfig.TILT_RIGHT_DOWN_POS);
        tiltRightDown.setName("tiltRightDown");
        this.addAction(tiltRightDown);
    }

}
