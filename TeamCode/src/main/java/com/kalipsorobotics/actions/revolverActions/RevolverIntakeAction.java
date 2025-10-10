package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.MotifColors;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KColorDetection;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class RevolverIntakeAction extends Action {

    private Revolver revolver;
    private RevColorSensorV3 sen1;
    private RevColorSensorV3 sen2;
    private RevColorSensorV3 sen3;
    private KServo revolverServo;

    private int count;

    public RevolverIntakeAction(Revolver revolver) {
        this.revolver = revolver;
        this.sen1 = revolver.getSen1();
        this.sen2 = revolver.getSen2();
        this.sen3 = revolver.getSen3();
        this.revolverServo = revolver.getRevolverServo();
        this.dependentActions.add(new DoneStateAction());

        count = 0;
    }

//    @Override
//    protected boolean checkDoneCondition() {
//        if (KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()) != MotifColors.NONE && KColorDetection.detectColor("revColor2", sen2, revolver.getOpModeUtilities()) != MotifColors.NONE && KColorDetection.detectColor("revColor3", sen3, revolver.getOpModeUtilities()) != MotifColors.NONE) {
//            revolver.setColorSet(0, KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()));
//            revolver.setColorSet(1, KColorDetection.detectColor("revColor2", sen2, revolver.getOpModeUtilities()));
//            revolver.setColorSet(2, KColorDetection.detectColor("revColor3", sen3, revolver.getOpModeUtilities()));
//            isDone = true;
//        }
//        return isDone;
//    }

    @Override
    protected void update() {
        if (!hasStarted) {
            if (KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()) != MotifColors.NONE && KColorDetection.detectColor("revColor2", sen2, revolver.getOpModeUtilities()) != MotifColors.NONE) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_2);
            } else if (KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()) != MotifColors.NONE) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_1);
            } else {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_0);
            }
        }

        if (KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()) != MotifColors.NONE) {
            count++;
            if (count == 1) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_1);
            } else if (count == 2) {
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_2);
            }
        }
        if (KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()) != MotifColors.NONE && KColorDetection.detectColor("revColor2", sen2, revolver.getOpModeUtilities()) != MotifColors.NONE && KColorDetection.detectColor("revColor3", sen3, revolver.getOpModeUtilities()) != MotifColors.NONE) {
            revolver.setColorSet(0, KColorDetection.detectColor("revColor1", sen1, revolver.getOpModeUtilities()));
            revolver.setColorSet(1, KColorDetection.detectColor("revColor2", sen2, revolver.getOpModeUtilities()));
            revolver.setColorSet(2, KColorDetection.detectColor("revColor3", sen3, revolver.getOpModeUtilities()));
            isDone = true;
        }
    }

}
