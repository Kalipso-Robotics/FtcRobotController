package com.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class DrivetrainConfigV2 {

    public static final double MM_PER_TICK = 0.04977974424; //Effective Constant  DistanceMM/Ticks


    //  Calibration Instruction
//  (LeftTicks-RightTicks) * MM_PER_TICKS/Angle = TRACK_WIDTH_MM
    public static final double TRACK_WIDTH_MM = 310.1446862;


    //  Calibration Instruction **Y change should be ~0**
//  (BackTicks * MM_PER_TICK)/Angle = BACK_DISTANCE_TO_MID_ROBOT_MM
    public static final double BACK_DISTANCE_TO_MID_ROBOT_MM = -91.19632916;


    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;


}
