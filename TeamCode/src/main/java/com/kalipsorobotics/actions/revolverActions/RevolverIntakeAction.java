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

        if (revolverServo.isDone() & hasStarted) {
            detectColorsAction.setIsDone(false);
            detectColorsAction.updateCheckDone();
        }

        if (!hasStarted) {
            hasStarted = true;
            detectColorsAction.setIsDone(false);
            detectColorsAction.updateCheckDone();
        }

        if (detectColorsAction.isFull()) {
            KLog.d("teleoprevolver", "done revolver intake");
            isDone = true;
        } else if (revolverServo.isDone() && !(detectColorsAction.getFrontColor() == MotifColor.NONE)){
            MotifColor[] motifColors = {detectColorsAction.getFrontColor(), detectColorsAction.getBrightColor(), detectColorsAction.getBLeftColor()};
            int emptySlotSensorIndex = Arrays.asList(motifColors).indexOf(MotifColor.NONE); //todo make it find the closest one
            KLog.d("teleoprevolver", "sensor list " + Arrays.asList(motifColors));
            int currentTrayIndex = revolver.getCurrentRevolverServoIndex();
            KLog.d("teleoprevolver", "empty sensor index " + emptySlotSensorIndex);
            KLog.d("teleoprevolver", "current tray index " + currentTrayIndex);
//            int indDiff = currentTrayIndex - emptySlotSensorIndex;

            int moveTo = (currentTrayIndex + emptySlotSensorIndex) % 3;
            KLog.d("teleoprevolver", "move to " + moveTo);

            revolver.moveToIndex(moveTo);
        }

    }
}
