package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Revolver {
    private static Revolver single_instance = null;

    private final OpModeUtilities opModeUtilities;

    public Servo revolverServo;

    public RevColorSensorV3 sen1;
    public RevColorSensorV3 sen2;
    public RevColorSensorV3 sen3;

    public static final double REVOLVER_INDEX_0 = 0.0; //todo set
    public static final double REVOLVER_INDEX_1 = 0.0; //todo set
    public static final double REVOLVER_INDEX_2 = 0.0; //todo set

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
        revolver.revolverServo = hardwareMap.servo.get("revolver");
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
    }
}
