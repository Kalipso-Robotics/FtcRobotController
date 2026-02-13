package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PIDController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;

public abstract class DriveTrainAction extends Action {

    public abstract PIDController getPidController();
    public abstract double getRemainingDistance();

    public abstract double getTarget();

    public abstract double getDuration();

    public abstract double getOvershoot();

    public abstract void setPidController(PIDController controller);
}
