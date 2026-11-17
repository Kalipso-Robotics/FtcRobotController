package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;

// Pattern: DriveController (actions/drivetrain)
public interface IntakeController {
    /** Call once per loop. */
    void update();

    /** The side currently chosen to intake, for telemetry and logs. */
    BeamBreakPosition.IntakeSide getSelectedSide();
}
