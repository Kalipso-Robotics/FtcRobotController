package com.kalipsorobotics.utilities;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.MotifColor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class MotifDetectionAction extends Action {
    private final MotifCamera motifCamera;

    public MotifDetectionAction(MotifCamera motifCamera) {
        this.motifCamera = motifCamera;
    }

    public MotifCamera getMotifCamera() {
        return motifCamera;
    }
    private int getObiliskID() {
        List<AprilTagDetection> detections = motifCamera.getAprilTagProcessor().getDetections();;

        return detections.get(0).id;
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
