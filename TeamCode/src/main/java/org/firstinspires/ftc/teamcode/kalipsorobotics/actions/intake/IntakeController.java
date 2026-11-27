package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;

// Pattern: DriveController (actions/drivetrain)
public interface IntakeController {
    /** Call once per loop. */
    void update();

    /** True once the intake has filled up (latched until clearFull). */
    boolean isFull();

    /** The balls left (shot, pass, reverse): forget the full latch and intake again. */
    void clearFull();

    /** The side currently chosen to intake, for telemetry and logs. */
    IntakeSide getSelectedSide();
}
