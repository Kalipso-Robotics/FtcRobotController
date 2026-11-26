package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;

@Config
public class ModuleConfig {


    //========STOPPER========
    public static final double STOPPER_SERVO_CLOSED_POS = 0.478; // TODO-measure
    public static final double STOPPER_SERVO_OPEN_POS = 0.41; // TODO-measure

    //========INTAKE========
    // Per-side settings (pivot positions, motor direction, feed gains): IntakeConfig.SIDE_A / SIDE_B.
    // Intake/transfer motor: goBILDA 5203 1620 rpm (3.7:1), encoder 28 * 3.7 = 103.8 ticks per output rev -> ~27 RPS free
    // at full power. Not read from the hub config (its motor type was wrong). Check: IntakeTest RT full, no balls, ~26 RPS.
    public static final double INTAKE_TICKS_PER_REV = 103.8;

    //========FLOWER========
    // Which intake side carries the flower arm. Changing sides is this one line.
    public static final BeamBreakPosition.IntakeSide FLOWER_SIDE = BeamBreakPosition.IntakeSide.B; // back (non-odometry side)


    //========BASIC TELEOP (test/BasicTeleop, manual control)========
    public static double BASIC_SHOOTER_STEP = 0.05; // TODO-measure: shooter power per dpad up/down
    public static double BASIC_SERVO_STEP = 0.01; // TODO-measure: servo step per dpad right/left
    public static double BASIC_SERVO_FINE_STEP = 0.001; // TODO-measure: servo step with A held
    public static double BASIC_DRIVE_SCALE = 1.0; // TODO-measure: drive power scale
}
