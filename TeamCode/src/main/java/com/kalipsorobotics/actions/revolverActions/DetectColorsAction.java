package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KColorDetection;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class DetectColorsAction extends Action {

    private Revolver revolver;
    private RevColorSensorV3 sen1;
    private RevColorSensorV3 sen2;
    private RevColorSensorV3 sen3;

    public DetectColorsAction(Revolver revolver) {
        this.revolver = revolver;
        this.sen1 = revolver.sen1;
        this.sen2 = revolver.sen2;
        this.sen3 = revolver.sen3;
        this.dependentActions.add(new DoneStateAction());
    }

    @Override
    protected boolean checkDoneCondition() {
        if (hasStarted) {
            isDone = true;
        }
        return isDone;
    }

    @Override
    protected void update() {
        if(!hasStarted) {
            revolver.setColorSet(0, KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()));
            revolver.setColorSet(1, KColorDetection.detectColor("revColor2", sen2, revolver.getOpModeUtilities()));
            revolver.setColorSet(2, KColorDetection.detectColor("revColor3", sen3, revolver.getOpModeUtilities()));
            hasStarted = true;
        }
    }
}
