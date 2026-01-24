package com.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ModuleConfig {


    //========BRAKE========
    public static final double RELEASE_BRAKE_POS = 0.40;
    public static final double ACTIVATE_BRAKE_POS = 0.65;

    //========STOPPER========
    public static final double STOPPER_SERVO_CLOSED_POS = 0.5;
    public static final double STOPPER_SERVO_OPEN_POS = 0.35;
}
