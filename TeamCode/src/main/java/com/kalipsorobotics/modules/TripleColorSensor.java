package com.kalipsorobotics.modules;

import static com.kalipsorobotics.modules.ColorSensorPosition.BLEFT;
import static com.kalipsorobotics.modules.ColorSensorPosition.BRIGHT;
import static com.kalipsorobotics.modules.ColorSensorPosition.FRONT;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import java.util.HashMap;

public class TripleColorSensor {
    private final RevColorSensorV3 bLeft;
    private final RevColorSensorV3 bRight;
    private final RevColorSensorV3 front;

    HashMap<ColorSensorPosition, RevColorSensorV3> sensors;

    public TripleColorSensor(OpModeUtilities opModeUtilities) {
        this.bLeft = opModeUtilities.getHardwareMap().get(RevColorSensorV3.class, "bLeftColor");
        this.bRight = opModeUtilities.getHardwareMap().get(RevColorSensorV3.class, "bRightColor");
        this.front = opModeUtilities.getHardwareMap().get(RevColorSensorV3.class, "frontColor");

        this.sensors = new HashMap<>();
        sensors.put(BLEFT, bLeft);
        sensors.put(BRIGHT, bRight);
        sensors.put(FRONT, front);
    }

    public RevColorSensorV3 getbLeft() {
        return bLeft;
    }

    public RevColorSensorV3 getbRight() {
        return bRight;
    }

    public RevColorSensorV3 getFront() {
        return front;
    }

    public HashMap<ColorSensorPosition, RevColorSensorV3> getSensors() {
        return sensors;
    }
}
