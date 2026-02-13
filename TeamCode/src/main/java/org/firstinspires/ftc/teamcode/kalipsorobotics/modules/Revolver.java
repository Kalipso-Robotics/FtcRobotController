package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

public class Revolver {
    private final OpModeUtilities opModeUtilities;

    private final KServo revolverServo;

    public static final double REVOLVER_INDEX_0 = 0.1; //+ for counterclockwise
    public static final double  REVOLVER_INDEX_1 = 0.5;
    public static final double REVOLVER_INDEX_2 = 0.9;

    private MotifColor[] colorSet = new MotifColor[3];

    public Revolver(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        Servo servo = opModeUtilities.getHardwareMap().servo.get("revolver");
        this.revolverServo = new KServo(servo, 60.0/0.35, 242, 0, false);
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

    public void moveToIndex(int i) {
        if (i == 0) {
            revolverServo.setTargetPosition(REVOLVER_INDEX_0);
        } else if (i == 1) {
            revolverServo.setTargetPosition(REVOLVER_INDEX_1);
        } else if (i == 2) {
            revolverServo.setTargetPosition(REVOLVER_INDEX_2);
        }
    }

}
