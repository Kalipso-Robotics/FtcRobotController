package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KServo;

import java.util.Arrays;

public class RevolverMoveToColorAction extends Action {
    private Revolver revolver;
    private KServo revolverServo;
    MotifColor shootColor;
    DetectColorsAction detectColorsAction;

    public RevolverMoveToColorAction(Revolver revolver, MotifColor shootColor, DetectColorsAction detectColorsAction) {
        this.revolver = revolver;
        this.revolverServo = revolver.getRevolverServo();
        this.shootColor = shootColor;
        this.dependentActions.add(new DoneStateAction());
        this.detectColorsAction = detectColorsAction;
    }

    @Override
    protected void update() {
        if (revolverServo.isDone()) {
            detectColorsAction.setIsDone(false);
            detectColorsAction.updateCheckDone();

            MotifColor[] colorSet = {detectColorsAction.getFrontColor(), detectColorsAction.getBrightColor(), detectColorsAction.getBLeftColor()};

            if(shootColor != colorSet[0]) {
                int currentRevolverIndex = revolver.getCurrentRevolverServoIndex();
                MotifColor[] transformedColorSet = detectColorsAction.transformColorSetToTray(colorSet, currentRevolverIndex);

                int turnToIndex = Arrays.asList(transformedColorSet).indexOf(shootColor); //todo make it find the closest one | wait laing wouldnt all the positions always be one turn away

                if (turnToIndex == -1) {
                    if (shootColor == MotifColor.PURPLE) {
                        turnToIndex = Arrays.asList(transformedColorSet).indexOf(MotifColor.GREEN);
                    } else if (shootColor == MotifColor.GREEN) {
                        turnToIndex = Arrays.asList(transformedColorSet).indexOf(MotifColor.PURPLE);
                    }
                }

                revolver.moveToIndex(turnToIndex);

//            int indDiff = revolver.getCurrentRevolverServoIndex() - currentRevolverIndex;
//            revolver.setColorSet(detectColorsAction.transformColorSetToTray(colorSet, indDiff));
            }
            isDone = true;
        }
    }
}
