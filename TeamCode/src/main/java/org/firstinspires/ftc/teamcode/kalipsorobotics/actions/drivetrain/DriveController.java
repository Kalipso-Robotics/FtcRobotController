package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface DriveController {
    void move(Gamepad gamepad);
    void setPowerCoefficient(double powerCoefficient);
    double getPowerCoefficient();
}
