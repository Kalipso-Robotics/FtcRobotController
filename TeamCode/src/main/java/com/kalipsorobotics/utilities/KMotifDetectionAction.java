package com.kalipsorobotics.utilities;

import com.kalipsorobotics.actions.actionUtilities.Action;
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
        return obiliskDetection.getExpectedMotifPattern(getObiliskID());
    }

    @Override
    protected boolean isUpdateDone() {
        isDone = obiliskDetection.isObeliskVisible();
        return isDone;
    }
    //TODO think about auto implementation + what to do if no pattern detected
    //TODO timeout if nothing detected

    @Override
    protected void update() {
        obiliskDetection.refreshMotifPattern();
        isDone = true;
    }
}
