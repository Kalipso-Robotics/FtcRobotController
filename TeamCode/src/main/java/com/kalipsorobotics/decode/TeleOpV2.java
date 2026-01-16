package com.kalipsorobotics.decode;

import com.kalipsorobotics.decode.configs.AprilTagConfig;
import com.kalipsorobotics.decode.configs.AprilTagConfigV2;
import com.kalipsorobotics.decode.configs.DrivetrainConfig;
import com.kalipsorobotics.decode.configs.DrivetrainConfigV2;
import com.kalipsorobotics.decode.configs.ModuleConfig;
import com.kalipsorobotics.decode.configs.ModuleConfigV2;
import com.kalipsorobotics.decode.configs.ShooterConfig;
import com.kalipsorobotics.decode.configs.ShooterConfigV2;
import com.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import com.kalipsorobotics.decode.configs.ShooterInterpolationConfigV2;
import com.kalipsorobotics.decode.configs.V2ConfigHelper;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpV2 extends TeleOp {

    @Override
    protected void initializeRobotConfig() {
        super.initializeRobotConfig();
        V2ConfigHelper.configRobotV2();
    }


}
