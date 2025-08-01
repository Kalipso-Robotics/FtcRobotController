package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class GoBildaOdoModule {

    private static GoBildaOdoModule single_instance = null;

    public  GoBildaPinpointDriver goBildaPinpointDriver;

    public GoBildaOdoModule(OpModeUtilities opModeUtilities) {

        goBildaPinpointDriver = opModeUtilities.getHardwareMap().get(GoBildaPinpointDriver.class, "goBildaOdo");

    }

    public static synchronized GoBildaOdoModule getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new GoBildaOdoModule(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, GoBildaOdoModule goBildaOdoModule) {
        goBildaOdoModule.goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "goBildaOdo");
    }

    public GoBildaPinpointDriver getGoBildaPinpointDriver() {
        return goBildaPinpointDriver;
    }
}
