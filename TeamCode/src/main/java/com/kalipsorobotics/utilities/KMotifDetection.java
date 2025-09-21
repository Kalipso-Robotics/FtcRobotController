package com.kalipsorobotics.utilities;

import com.kalipsorobotics.cameraVision.MotifColor;
import com.kalipsorobotics.cameraVision.ObiliskDetection;

public class KMotifDetection {
    private final ObiliskDetection obiliskDetection;
    public KMotifDetection(ObiliskDetection obiliskDetection) {
        this.obiliskDetection = obiliskDetection;
    }


    public ObiliskDetection getObiliskDetection() {
        return obiliskDetection;
    }
    public int getObiliskID() {
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
    public MotifColor getFirstMotifColor() {
        return getMotifPattern().top;
    }
    public MotifColor getSecondMotifColor() {
        return getMotifPattern().middle;
    }
    public MotifColor getThirdMotifColor() {
        return getMotifPattern().bottom;
    }

}
