package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Field;

public class DetectPropPositionAction extends Action {
    ElapsedTime elapsedTime;
    VisionPortalProcessor visionPortalProcessor;
    FieldPosition fieldPosition;
    PropDetector.PROP_POSITION position;
    Boolean longPath;
    int visionTimeout = 500; // timeout detection after 2 seconds
    double time;
    boolean timeStarted = false;

    public DetectPropPositionAction(Action dependentAction, VisionPortalProcessor visionProcessor, FieldPosition fieldPosition, Boolean longPath) {
        elapsedTime = new ElapsedTime();
        this.visionPortalProcessor = visionProcessor;
        this.fieldPosition = fieldPosition;
        this.dependentAction = dependentAction;
        this.longPath = longPath;
        position = PropDetector.PROP_POSITION.UNKNOWN;
    }

    public DetectPropPositionAction(VisionPortalProcessor visionPortalProcessor, FieldPosition fieldPosition, Boolean longPath) {
        Log.d("vision", "action: constructing");
        elapsedTime = new ElapsedTime();
        Log.d("vision", "action: about to set vision portal processor field");
        this.visionPortalProcessor = visionPortalProcessor;
        Log.d("vision", "action: set up vision portal processor field");
        this.fieldPosition = fieldPosition;
        this.dependentAction = new DoneStateAction();
        this.longPath = longPath;
        position = PropDetector.PROP_POSITION.UNKNOWN;
        Log.d("vision", "action: finished constructing");
    }

    @Override
    boolean checkDoneCondition() {
        if (position == PropDetector.PROP_POSITION.UNDETECTED || position == PropDetector.PROP_POSITION.UNKNOWN) {
            return false;
        } else {
            fieldPosition.setMarkerLocation(position);
            if (fieldPosition.isRedAlliance) {
                fieldPosition.setWantedAprTagId(position, PropDetector.ALLIANCE_COLOR.RED);
            } else {
                fieldPosition.setWantedAprTagId(position, PropDetector.ALLIANCE_COLOR.BLUE);
            }

            visionPortalProcessor.getVisionPortal().setProcessorEnabled(visionPortalProcessor.getPropProcessor(), false);
            Log.d("vision", "action: finished action");
            return true;
        }
    }

    @Override
    void update() {
        Log.d("vision", "action: entered action update");

        if(!timeStarted) {
            Log.d("vision","action: started detecting");
            time = elapsedTime.milliseconds();
            position = PropDetector.PROP_POSITION.UNDETECTED;
            visionPortalProcessor.getVisionPortal().setProcessorEnabled(visionPortalProcessor.getPropProcessor(), true);
            timeStarted = true;
        }

        //detect marker position
        int i =  0;
        if (position == PropDetector.PROP_POSITION.UNDETECTED || position == PropDetector.PROP_POSITION.UNKNOWN) {
            i++;
            Log.d("vision", "action: " + i + " undetected marker, keep looking" + visionPortalProcessor.getVisionPortal().getCameraState());
            position = visionPortalProcessor.getPropProcessor().getPosition();
            Log.d("vision", "action: get position is" + visionPortalProcessor.getPropProcessor().getPosition());
            //Log.d("color detection", String.valueOf(propProcessor.avgCenterCb));
            //Log.d("color detection", String.valueOf(markerProcessor.avgRightCb));
            //Log.d("color detection", String.valueOf(propProcessor.avgCenterCr));
            Log.d("vision", "action: elapsed time" + String.valueOf(elapsedTime.milliseconds()));
            if ((elapsedTime.milliseconds() > time + visionTimeout) && position == PropDetector.PROP_POSITION.UNDETECTED) {
                position = PropDetector.PROP_POSITION.CENTER;
                Log.d("vision", "action: detected time out. Picking CENTER");
            }
        }

        Log.d("vision", "action: done process, detected at" + String.valueOf(elapsedTime.milliseconds()));

        //save marker position and apriltag position in robot class
        //todo
//        setWantedAprTagId(position, visionProcessor.isRedAlliance ? MarkerDetector.ALLIANCE_COLOR.RED : MarkerDetector.ALLIANCE_COLOR.BLUE);
//        setSecondWantedTagId();

        //print position
        Log.d("vision", "action: done process, detected position: " + position);
        visionPortalProcessor.getOpModeUtilities().getTelemetry().addData("position", position);
        visionPortalProcessor.getOpModeUtilities().getTelemetry().update();
    }

    public FieldPosition.PROP_LOCATION getPropLocation() {
        return fieldPosition.getPropLocation();
    }
}