package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class GoBildaOdoModule {

    private static GoBildaOdoModule single_instance = null;

    private GoBildaPinpointDriver goBildaPinpointDriver;

    public GoBildaOdoModule(OpModeUtilities opModeUtilities) {

        goBildaPinpointDriver = opModeUtilities.getHardwareMap().get(GoBildaPinpointDriver.class, "goBildaOdometry");
        goBildaPinpointDriver.resetPosAndIMU();
        opModeUtilities.getOpMode().sleep(500);
        goBildaPinpointDriver.recalibrateIMU();
        opModeUtilities.getOpMode().sleep(500);
        goBildaPinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        goBildaPinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
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
        goBildaOdoModule.goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "goBildaOdometry");
    }

    public GoBildaPinpointDriver getGoBildaPinpointDriver() {
        return goBildaPinpointDriver;
    }
}
