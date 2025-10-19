package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KServo;

import java.util.Arrays;

public class RevolverIntakeAction extends Action {

    private final Revolver revolver;

    private final KServo revolverServo;

    private final DetectColorsAction detectColorsAction;


    public RevolverIntakeAction(Revolver revolver, TripleColorSensor colorSensors) {
        this.revolver = revolver;
        this.revolverServo = revolver.getRevolverServo();
        this.dependentActions.add(new DoneStateAction());
        this.detectColorsAction = new DetectColorsAction(colorSensors, revolver.getOpModeUtilities());
    }


    @Override
    protected void update() {

        if (revolverServo.isDone()) {
            detectColorsAction.setIsDone(false);
            detectColorsAction.updateCheckDone();
        }

        MotifColor[] motifColors = {detectColorsAction.getFrontColor(), detectColorsAction.getBrightColor(), detectColorsAction.getBLeftColor()};

        if (!hasStarted) {
            hasStarted = true;
        }

        if (detectColorsAction.isFull()) {
            KLog.d("teleoprevolver", "done revolver intake");
            isDone = true;
        } else if (!(detectColorsAction.getFrontColor() == MotifColor.NONE)){
            int emptySlotSensorIndex = Arrays.asList(motifColors).indexOf(MotifColor.NONE); //todo make it find the closest one
            int currentTrayIndex = revolver.getCurrentRevolverServoIndex();
            KLog.d("teleoprevolver", "done revolver intake");
//            int indDiff = currentTrayIndex - emptySlotSensorIndex;

            int moveTo = (currentTrayIndex + emptySlotSensorIndex) % 3;

            revolver.moveToIndex(moveTo);
        }

    }
}
