package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KColor;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.K;

public class Revolver {
    private static Revolver single_instance = null;

    private final OpModeUtilities opModeUtilities;

    public KServo revolverServo;

    public RevColorSensorV3 sen1;
    public RevColorSensorV3 sen2;
    public RevColorSensorV3 sen3;

    public static final double REVOLVER_INDEX_0 = 0.195;
    public static final double REVOLVER_INDEX_1 = 0.58;
    public static final double REVOLVER_INDEX_2 = 0.965;

    private KColor.Color[] colorSet = new KColor.Color[3];

    private Revolver(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        resetHardwareMap(opModeUtilities.getHardwareMap(), this);
    }

    public static synchronized Revolver getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new Revolver(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, Revolver revolver) {
        Servo servo = hardwareMap.servo.get("revolver");
        revolver.revolverServo = new KServo(servo, KServo.AXON_MAX_SPEED,242, 0, false);
        revolver.sen1 = hardwareMap.get(RevColorSensorV3.class, "revColor1");
        revolver.sen2 = hardwareMap.get(RevColorSensorV3.class, "revColor2");
        revolver.sen3 = hardwareMap.get(RevColorSensorV3.class, "revColor3");
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
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

    public void setColorSet(int index, KColor.Color color) {
        this.colorSet[index] = color;
    }

    public void setColorSet(KColor.Color[] color) {
        this.colorSet = colorSet;
    }

    public KColor.Color[] getColorSet() {
        setColorSet(0, KColor.classify(sen1));
        setColorSet(1, KColor.classify(sen2));
        setColorSet(2, KColor.classify(sen3));
        return colorSet;
    }

    public static KColor.Color[] transformColorSetToTray(KColor.Color[] colorSet, int currentRevolverIndex) {
        KColor.Color[] transformedColorSet = new KColor.Color[3];

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
