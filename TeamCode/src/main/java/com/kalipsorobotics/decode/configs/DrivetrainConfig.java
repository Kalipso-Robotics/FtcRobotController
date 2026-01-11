package com.kalipsorobotics.decode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class DrivetrainConfig {
    public static double MM_PER_TICK = 0.0744953182; //Effective Constant  DistanceMM/Ticks


//  Calibration Instruction
//  (LeftTicks-RightTicks) * MM_PER_TICKS/Angle = TRACK_WIDTH_MM
    public static double TRACK_WIDTH_MM = 297.5;


//  Calibration Instruction **Y change should be ~0**
//  (BackTicks * MM_PER_TICK)/Angle = BACK_DISTANCE_TO_MID_ROBOT_MM
    public static double BACK_DISTANCE_TO_MID_ROBOT_MM = -90;


    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

}

