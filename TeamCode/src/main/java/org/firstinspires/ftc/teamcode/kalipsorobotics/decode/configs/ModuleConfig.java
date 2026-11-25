package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;

@Config
public class ModuleConfig {


    //========STOPPER========
    public static final double STOPPER_SERVO_CLOSED_POS = 0.478; // TODO-measure
    public static final double STOPPER_SERVO_OPEN_POS = 0.41; // TODO-measure

    //========INTAKE========
    // Side A is the front (+X, odometry side), side B the back.
    public static final double INTAKE_PIVOT_A_DOWN_POS = 0.68;
    public static final double INTAKE_PIVOT_A_UP_POS = 0.58;
    // 0.44 ALL THE WAY UP FOR A(1) servo for INNIT_POS
    public static final double INTAKE_PIVOT_B_DOWN_POS = 0.3;
    public static final double INTAKE_PIVOT_B_UP_POS = 0.420;
    // 0.55 ALL THE WAY UP for B (2) servo for INNIT POS
    public static final DcMotorSimple.Direction INTAKE_MOTOR_A_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction INTAKE_MOTOR_B_DIRECTION = DcMotorSimple.Direction.FORWARD; // + power = intake in on both sides; fix spin here, not with a minus sign in callers
    // Intake/transfer motor: goBILDA 5203 1620 rpm (3.7:1), encoder 28 * 3.7 = 103.8 ticks per output rev -> ~27 RPS free
    // at full power. Not read from the hub config (its motor type was wrong). Check: IntakeTest RT full, no balls, ~26 RPS.
    public static final double INTAKE_TICKS_PER_REV = 103.8;

    //========FLOWER========
    // Which intake side carries the flower arm. Changing sides is this one line.
    public static final BeamBreakPosition.IntakeSide FLOWER_SIDE = BeamBreakPosition.IntakeSide.A;


    //========BASIC TELEOP (test/BasicTeleop, manual control)========
    public static double BASIC_SHOOTER_STEP = 0.05; // TODO-measure: shooter power per dpad up/down
    public static double BASIC_SERVO_STEP = 0.01; // TODO-measure: servo step per dpad right/left
    public static double BASIC_SERVO_FINE_STEP = 0.001; // TODO-measure: servo step with A held
    public static double BASIC_DRIVE_SCALE = 1.0; // TODO-measure: drive power scale
}
