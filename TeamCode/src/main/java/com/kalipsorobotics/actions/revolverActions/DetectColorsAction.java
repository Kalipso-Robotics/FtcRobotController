package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class DetectColorsAction extends Action {

    private Revolver revolver;
    private RevColorSensorV3 sen1;
    private RevColorSensorV3 sen2;
    private RevColorSensorV3 sen3;

    public DetectColorsAction(Revolver revolver) {
        this.revolver = revolver;
        this.sen1 = revolver.getSen1();
        this.sen2 = revolver.getSen2();
        this.sen3 = revolver.getSen3();
        this.dependentActions.add(new DoneStateAction());
    }

    @Override
    protected boolean updateIsDone() {
        if (hasStarted) {
            isDone = true;
        }
        return isDone;
    }

    @Override
    protected void update() {
        if(!hasStarted) {
            revolver.setColorSet(0, KColor.classify(sen1));
            revolver.setColorSet(1, KColor.classify(sen2));
            revolver.setColorSet(2, KColor.classify(sen3));
            hasStarted = true;
        }
    }
}
