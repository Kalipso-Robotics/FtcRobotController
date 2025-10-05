package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

public class RevolverMotifTurnAction extends Action {

    private RevColorSensorV3 sen1;
    private RevColorSensorV3 sen2;
    private RevColorSensorV3 sen3;
    private Revolver revolver;
    private Servo revolverServo;
    private KColor.Color[] colorSet;

    private ObiliskDetection.MotifPattern motifPattern;

    public RevolverMotifTurnAction(Revolver revolver, ObiliskDetection obiliskDetection, DetectColorsAction detectColorsAction) {
        this.sen1 = revolver.sen1;
        this.sen2 = revolver.sen2;
        this.sen3 = revolver.sen3;
        this.revolver = revolver;
        this.revolverServo = revolver.revolverServo;
        this.motifPattern = obiliskDetection.getObeliskMotifPattern();
        this.colorSet = revolver.getColorSet();
        this.dependentActions.add(new DoneStateAction());
    }

    @Override
    protected boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    protected void update() {
        if (!hasStarted) {
        }

        if (motifPattern.equals(KColor.Color.GREEN, KColor.Color.PURPLE, KColor.Color.PURPLE)) {
            if (colorSet[0] == KColor.Color.GREEN) {
                //send ball
            } else {

            }
        }
    }
}
