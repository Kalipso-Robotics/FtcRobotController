package com.kalipsorobotics.utilities;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.cameraVision.MotifColor;
import com.kalipsorobotics.cameraVision.ObiliskDetection;

public class KMotifDetectionAction extends Action {
    private final ObiliskDetection obiliskDetection;
    public KMotifDetectionAction(ObiliskDetection obiliskDetection) {
        this.obiliskDetection = obiliskDetection;
    }


    public ObiliskDetection getObiliskDetection() {
        return obiliskDetection;
    }
    private int getObiliskID() {
        return obiliskDetection.getObeliskId();
        /*
        21 = GPP
        22 = PGP
        23 = PPG
         */
    }
    public ObiliskDetection.MotifPattern getMotifPattern() {
        return obiliskDetection.getObeliskMotifPattern();
    }
    //TODO
    @Override
    protected boolean checkDoneCondition() {
        return false;
    }

    @Override
    protected void update() {
        obiliskDetection.refreshMotifPattern();
        isDone = true;
    }
}
