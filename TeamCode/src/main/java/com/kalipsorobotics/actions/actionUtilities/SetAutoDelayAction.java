package com.kalipsorobotics.actions.actionUtilities;

import com.kalipsorobotics.utilities.KGamePad;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SetAutoDelayAction extends Action {

    OpModeUtilities opModeUtilities;
    KGamePad kGamePad1;
    int time = 0;

    public SetAutoDelayAction(OpModeUtilities opModeUtilities, Gamepad gamepad) {
        kGamePad1 = new KGamePad(gamepad);
        this.opModeUtilities = opModeUtilities;
    }

    public int getTimeMs() {
        return time*1000;
    }

    @Override
    protected boolean updateIsDone() {
        if(isDone) {
            return true;
        }

        if (kGamePad1.isLeftBumperFirstPressed() && kGamePad1.isRightBumperFirstPressed()){
            opModeUtilities.getTelemetry().addLine("CONFIRMED DELAY");
            opModeUtilities.getTelemetry().addData("DELAY TIME IN MS ", getTimeMs());
            opModeUtilities.getTelemetry().update();
            isDone = true;
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void update() {
        if (kGamePad1.isDpadLeftFirstPressed()) {
            time -= 1;
        } else if (kGamePad1.isDpadRightFirstPressed()) {
            time += 1;
        }

        if (time<0) {
            time = 0;
        }

        opModeUtilities.getTelemetry().addLine("INIT FINISHED");
        opModeUtilities.getTelemetry().addData("DELAY TIME IN MS ", getTimeMs());
        opModeUtilities.getTelemetry().update();
    }
}
