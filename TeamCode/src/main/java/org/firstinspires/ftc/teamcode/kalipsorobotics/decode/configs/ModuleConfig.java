package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;

@Config
public class ModuleConfig {


    //========STOPPER========
    public static final double STOPPER_SERVO_CLOSED_POS = 0.47; //0.49
    public static final double STOPPER_SERVO_OPEN_POS = 0.31; //0.35

    //========INTAKE========
    // Side A is the front (+X, odometry side), side B the back.
    public static final double INTAKE_PIVOT_A_DOWN_POS = 0.3; // TODO-measure
    public static final double INTAKE_PIVOT_A_UP_POS = 0.7; // TODO-measure
    public static final double INTAKE_PIVOT_B_DOWN_POS = 0.3; // TODO-measure
    public static final double INTAKE_PIVOT_B_UP_POS = 0.7; // TODO-measure
    public static final DcMotorSimple.Direction INTAKE_MOTOR_A_DIRECTION = DcMotorSimple.Direction.FORWARD; // TODO-measure
    public static final DcMotorSimple.Direction INTAKE_MOTOR_B_DIRECTION = DcMotorSimple.Direction.FORWARD; // TODO-measure

    //========FLOWER========
    // Which intake side carries the flower arm. Changing sides is this one line.
    public static final BeamBreakPosition.IntakeSide FLOWER_SIDE = BeamBreakPosition.IntakeSide.A;
    public static final double FLOWER_DOWN_POS = 0.8; // TODO-measure
    public static final double FLOWER_UP_POS = 0.2; // TODO-measure


    //========BASIC TELEOP (test/BasicTeleop, manual control)========
    public static double BASIC_SHOOTER_STEP = 0.05; // TODO-measure: shooter power per dpad up/down
    public static double BASIC_SERVO_STEP = 0.01; // TODO-measure: servo step per dpad right/left
    public static double BASIC_SERVO_FINE_STEP = 0.001; // TODO-measure: servo step with A held
    public static double BASIC_DRIVE_SCALE = 1.0; // TODO-measure: drive power scale
}
