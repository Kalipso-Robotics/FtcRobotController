package com.kalipsorobotics.actions.cameraVision;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.MotifColor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class MotifDetectionAction extends Action {
    private final MotifCamera motifCamera;
    private long startTime;
    public MotifDetectionAction(MotifCamera motifCamera) {
        this.motifCamera = motifCamera;
    }
    public void startSearching() {
        this.startTime = System.currentTimeMillis();
    }
    public MotifCamera getMotifCamera() {
        return motifCamera;
    }
    private int getObiliskID() {
        List<AprilTagDetection> detections = motifCamera.getAprilTagProcessor().getDetections();

        return detections.get(0).id;
    }
    public boolean hasTimedOut() {
        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - startTime;

        return elapsedTime >= 5000 && !MotifVisible();
    }
    public MotifCamera.MotifPattern getMotifPattern() {
        if (getObiliskID() == 21) {
            return new MotifCamera.MotifPattern(MotifColor.GREEN, MotifColor.PURPLE, MotifColor.PURPLE);
        }
        if (getObiliskID() == 22) {
            return new MotifCamera.MotifPattern(MotifColor.PURPLE, MotifColor.GREEN, MotifColor.PURPLE);
        }
        if (getObiliskID() == 23) {
            return new MotifCamera.MotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN);
        }
        else { //nothing seen
            return new MotifCamera.MotifPattern(MotifColor.NONE, MotifColor.NONE, MotifColor.NONE);
        }
    }
    private boolean MotifVisible() {
        return !motifCamera.getAprilTagProcessor().getDetections().isEmpty();
    }
    public void resumeStreaming() {
        motifCamera.getVisionPortal().resumeStreaming();
    }
    public void stopStreaming() {
        motifCamera.getVisionPortal().stopStreaming();
    }
    public void close() {
        motifCamera.getVisionPortal().close();
    }

    @Override
    protected boolean isUpdateDone() {
        isDone = (!getMotifPattern().equals(MotifColor.NONE, MotifColor.NONE, MotifColor.NONE));
        return isDone;
    }
    //TODO think about auto implementation + what to do if no pattern detected
    //TODO timeout if nothing detected

    @Override
    protected void update() {
        getObiliskID();
        isDone = true;
    }
}
