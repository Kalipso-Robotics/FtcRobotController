package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BallDetection;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import java.util.ArrayList;
import java.util.List;

public class PollenNectarDetectionAction extends Action {
    private long timeoutMs;
    private long startTimeMs = -1;
    private List<BallDetection> detections = new ArrayList<>();
    private boolean timedOut;
    private static final long MAX_DATA_AGE_MS = 200;
    private PollenNectarDetectionAction(double seconds){
        seconds*=1000;
        timeoutMs = (long) seconds;
        setName("pollenNectarDetection");
    }




    @Override
    protected void update() {
        

    }
}
