package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class RevolverIntakeAction extends Action {

    private final Revolver revolver;
    private final RevColorSensorV3 sen1;
    private final RevColorSensorV3 sen2;
    private final RevColorSensorV3 sen3;
    private final KServo revolverServo;

    private DetectColorsAction detectColorsAction;

    private final TripleColorSensor colorSensors;

    private int count;

    public RevolverIntakeAction(Revolver revolver, TripleColorSensor colorSensors) {
        this.revolver = revolver;
        this.sen1 = colorSensors.getbLeft();
        this.sen2 = colorSensors.getbRight();
        this.sen3 = colorSensors.getFront();
        this.revolverServo = revolver.getRevolverServo();
        this.dependentActions.add(new DoneStateAction());
        this.colorSensors = colorSensors;
        this.detectColorsAction = new DetectColorsAction(colorSensors, revolver.getOpModeUtilities());

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
        detectColorsAction.updateCheckDone();

        if (!hasStarted) {
            if (detectColorsAction.getFrontColor() != MotifColor.NONE && detectColorsAction.getBLeftColor() != MotifColor.NONE) {
                KLog.d("teleoprevolver", "begin revolver intake, move tray index to 2");
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_2);
            } else if (detectColorsAction.getFrontColor() != MotifColor.NONE) {
                KLog.d("teleoprevolver", "begin revolver intake, move tray index to 1");
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_1);
            } else {
                KLog.d("teleoprevolver", "begin revolver intake, move tray index to 0");
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_0);
            }
            hasStarted = true;
        }

        if (detectColorsAction.getFrontColor() != MotifColor.NONE) {
            KLog.d("teleoprevolver", "front has ball");
            count++;
            if (count == 1) {
                KLog.d("teleoprevolver", "move tray index to 1");
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_1);
            } else if (count == 2) {
                KLog.d("teleoprevolver", "move tray index to 2");
                revolverServo.setPosition(Revolver.REVOLVER_INDEX_2);
            }
        }

        if (detectColorsAction.getFrontColor() != MotifColor.NONE &&detectColorsAction.getBLeftColor() != MotifColor.NONE && detectColorsAction.getBrightColor() != MotifColor.NONE) {
            KLog.d("teleoprevolver", "done revolver intake");
//            revolver.setColorSet(0, detectColorsAction.getBLeftColor());
//            revolver.setColorSet(1, detectColorsAction.getBLeftColor());
//            revolver.setColorSet(2, detectColorsAction.getBrightColor());
            isDone = true;
        }
    }

}
