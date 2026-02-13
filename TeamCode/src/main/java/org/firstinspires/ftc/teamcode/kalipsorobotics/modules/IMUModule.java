package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.DrivetrainConfig.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

public class IMUModule {
    private static IMUModule single_instance = null;

    private final OpModeUtilities opModeUtilities;
    public IMU imu;
    //Left, Forward
    private IMUModule(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        resetHardwareMap(opModeUtilities.getHardwareMap(), this);
        int i = 0;
        while (i < 3) {
            i++;
            boolean isImusInitialized = imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            IMU_LOGO_FACING_DIRECTION,
                            IMU_USB_FACING_DIRECTION
                    )
            ));
            if (isImusInitialized) {
                break;
            } else {
                opModeUtilities.getOpMode().sleep(500);
            }
        }

        imu.resetYaw();
    }

    public static synchronized IMUModule getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new IMUModule(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, IMUModule imuModule) {
        imuModule.imu = hardwareMap.get(IMU.class, "imu");
    }

    public IMU getIMU() {
        return imu;
    }
}
