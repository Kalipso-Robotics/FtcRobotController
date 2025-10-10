package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

public class RevolverIntakeAction extends Action {

    private Revolver revolver;
    private RevColorSensorV3 sen1;
    private RevColorSensorV3 sen2;
    private RevColorSensorV3 sen3;
    private KServo revolverServo;

    private int count;

    public RevolverIntakeAction(Revolver revolver) {
        this.revolver = revolver;
        this.sen1 = revolver.sen1;
        this.sen2 = revolver.sen2;
        this.sen3 = revolver.sen3;
        this.revolverServo = revolver.revolverServo;
        this.dependentActions.add(new DoneStateAction());

        count = 0;
    }

    @Override
    protected boolean updateIsDone() {
        if (KColor.classify(sen1) != KColor.Color.NONE && KColor.classify(sen2) != KColor.Color.NONE && KColor.classify(sen3) != KColor.Color.NONE) {
            revolver.setColorSet(0, KColor.classify(sen1));
            revolver.setColorSet(1, KColor.classify(sen2));
            revolver.setColorSet(2, KColor.classify(sen3));
            isDone = true;
        }
        return isDone;
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            if (KColor.classify(sen1) != KColor.Color.NONE && KColor.classify(sen2) != KColor.Color.NONE) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_2);
            } else if (KColor.classify(sen1) != KColor.Color.NONE) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_1);
            } else {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_0);
            }
        }

        if (KColor.classify(sen1) != KColor.Color.NONE) {
            count++;
            if (count == 1) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_1);
            } else if (count == 2) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_2);
            }
        }
    }
}
