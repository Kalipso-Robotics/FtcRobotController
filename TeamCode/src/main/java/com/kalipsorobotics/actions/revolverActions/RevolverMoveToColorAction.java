package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import java.util.Arrays;

public class RevolverMoveToColorAction extends Action {
    private Revolver revolver;
    private KServo revolverServo;
    KColor.Color shootColor;

    public RevolverMoveToColorAction(Revolver revolver, KColor.Color shootColor) {
        this.revolver = revolver;
        this.revolverServo = revolver.getRevolverServo();
        this.shootColor = shootColor;
        this.dependentActions.add(new DoneStateAction());
    }

    @Override
    protected void update() {
        KColor.Color[] colorSet = revolver.getColorSet();

        if(shootColor != colorSet[0]) {
            int currentRevolverIndex = revolver.getCurrentRevolverServoIndex();
            KColor.Color[] transformedColorSet = Revolver.transformColorSetToTray(colorSet, currentRevolverIndex);

            int turnToIndex = Arrays.asList(transformedColorSet).indexOf(shootColor); //todo make it find the closest one | wait laing wouldnt all the positions always be one turn away
            revolverServo.setPosition(turnToIndex);

            int indDiff = revolver.getCurrentRevolverServoIndex() - currentRevolverIndex;

            revolver.setColorSet(Revolver.transformColorSetToTray(colorSet, indDiff));
        }

        isDone = true;
    }
}
