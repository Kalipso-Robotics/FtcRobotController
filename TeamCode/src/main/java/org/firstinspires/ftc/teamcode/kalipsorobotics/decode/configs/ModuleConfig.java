package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;

@Config
public class ModuleConfig {


    //========STOPPER========
    public static final double STOPPER_SERVO_CLOSED_POS = 0.47; // TODO-measure
    public static final double STOPPER_SERVO_OPEN_POS = 0.31; // TODO-measure

    //========INTAKE========
    // Side A is the front (+X, odometry side), side B the back.
    public static final double INTAKE_PIVOT_A_DOWN_POS = 0.68; // TODO-measure
    public static final double INTAKE_PIVOT_A_UP_POS = 0.58; // TODO-measure
    // 0.44 ALL THE WAY UP FOR A(1) servo for INNIT_POS
    public static final double INTAKE_PIVOT_B_DOWN_POS = 0.3; // TODO-measure
    public static final double INTAKE_PIVOT_B_UP_POS = 0.55; // TODO-measure
    // 0.55 ALL THE WAY UP for B (2) servo for INNIT POS
    public static final DcMotorSimple.Direction INTAKE_MOTOR_A_DIRECTION = DcMotorSimple.Direction.FORWARD; // TODO-measure
    public static final DcMotorSimple.Direction INTAKE_MOTOR_B_DIRECTION = DcMotorSimple.Direction.REVERSE; // + power = intake in on both sides; fix spin here, not with a minus sign in callers

    //========FLOWER========
    // Which intake side carries the flower arm. Changing sides is this one line.
    public static final BeamBreakPosition.IntakeSide FLOWER_SIDE = BeamBreakPosition.IntakeSide.A;


    //========BASIC TELEOP (test/BasicTeleop, manual control)========
    public static double BASIC_SHOOTER_STEP = 0.05; // TODO-measure: shooter power per dpad up/down
    public static double BASIC_SERVO_STEP = 0.01; // TODO-measure: servo step per dpad right/left
    public static double BASIC_SERVO_FINE_STEP = 0.001; // TODO-measure: servo step with A held
    public static double BASIC_DRIVE_SCALE = 1.0; // TODO-measure: drive power scale
}
