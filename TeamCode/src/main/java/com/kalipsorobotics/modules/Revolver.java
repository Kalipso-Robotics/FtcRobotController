package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KColorDetection;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Revolver {
    private final OpModeUtilities opModeUtilities;

    private final KServo revolverServo;

    private final RevColorSensorV3 sen1; // made private for encapsulation little k :)
    private final RevColorSensorV3 sen2;
    private final RevColorSensorV3 sen3;

    public static final double REVOLVER_INDEX_0 = 0.195;
    public static final double REVOLVER_INDEX_1 = 0.58;
    public static final double REVOLVER_INDEX_2 = 0.965;

    private MotifColors[] colorSet = new MotifColors[3];

    public Revolver(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        Servo servo = opModeUtilities.getHardwareMap().servo.get("revolver");
        this.revolverServo = new KServo(servo, KServo.AXON_MAX_SPEED, 242, 0, false);
        this.sen1 = opModeUtilities.getHardwareMap().get(RevColorSensorV3.class, "revColor1");
        this.sen2 = opModeUtilities.getHardwareMap().get(RevColorSensorV3.class, "revColor2");
        this.sen3 = opModeUtilities.getHardwareMap().get(RevColorSensorV3.class, "revColor3");
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public KServo getRevolverServo() {
        return revolverServo;
    }

    public RevColorSensorV3 getSen1() {
        return sen1;
    }

    public RevColorSensorV3 getSen2() {
        return sen2;
    }

    public RevColorSensorV3 getSen3() {
        return sen3;
    }

    public int getCurrentRevolverServoIndex() {
        if (revolverServo.getPosition() == REVOLVER_INDEX_0) {
            return 0;
        } else if (revolverServo.getPosition() == REVOLVER_INDEX_1) {
            return 1;
        } else if (revolverServo.getPosition() == REVOLVER_INDEX_2) {
            return 2;
        }

        return -1;
    }

    public void setColorSet(int index, MotifColors color) {
        this.colorSet[index] = color;
    }

    public void setColorSet(MotifColors[] color) {
        this.colorSet = colorSet;
    }

    public MotifColors[] getColorSet() {
        setColorSet(0, KColorDetection.detectColor("revColor1", sen1, opModeUtilities));
        setColorSet(1, KColorDetection.detectColor("revColor2", sen2, opModeUtilities));
        setColorSet(2, KColorDetection.detectColor("revColor3", sen3, opModeUtilities));
        return colorSet;
    }

    public static MotifColors[] transformColorSetToTray(MotifColors[] colorSet, int currentRevolverIndex) {
        MotifColors[] transformedColorSet = new MotifColors[3];

        //transformed color set holds color of ball in each revolver tray index,
        //rather than color above each color sensor index.
        if (currentRevolverIndex == 0) {
            transformedColorSet = colorSet;
        } else if (currentRevolverIndex == 1) {
            transformedColorSet[0] = colorSet[2];
            transformedColorSet[1] =  colorSet[0];
            transformedColorSet[2] = colorSet[1];
        } else if (currentRevolverIndex == 2) {
            transformedColorSet[0] = colorSet[1];
            transformedColorSet[1] =  colorSet[2];
            transformedColorSet[2] = colorSet[0];
        }

        return transformedColorSet;
    }
}
