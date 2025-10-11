package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

public class Revolver {
    private final OpModeUtilities opModeUtilities;

    private final KServo revolverServo;

    public static final double REVOLVER_INDEX_0 = 0.160;
    public static final double REVOLVER_INDEX_1 = 0.53;
    public static final double REVOLVER_INDEX_2 = 0.930;

    private MotifColor[] colorSet = new MotifColor[3];

    public Revolver(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        Servo servo = opModeUtilities.getHardwareMap().servo.get("revolver");
        this.revolverServo = new KServo(servo, KServo.AXON_MAX_SPEED, 242, 0, false);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public KServo getRevolverServo() {
        return revolverServo;
    }

    public int getCurrentRevolverServoIndex() {
        double currentPosition = revolverServo.getPosition();
        int decimalPlaces = 3;

        double factor = Math.pow(10, decimalPlaces);

        double roundedPosition = Math.round(currentPosition * factor) / factor;

        if (roundedPosition == REVOLVER_INDEX_0) {
            return 0;
        } else if (roundedPosition == REVOLVER_INDEX_1) {
            return 1;
        } else if (roundedPosition == REVOLVER_INDEX_2) {
            return 2;
        }

        return -1;
    }

    public void setColorSet(int index, MotifColor color) {
        this.colorSet[index] = color;
    }

    public void setColorSet(MotifColor[] color) {
        this.colorSet = colorSet;
    }

}
