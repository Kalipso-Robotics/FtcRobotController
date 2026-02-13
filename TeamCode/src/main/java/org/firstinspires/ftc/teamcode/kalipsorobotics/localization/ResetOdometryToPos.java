package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToPos extends Action {

    Position zeroPos;

    public ResetOdometryToPos(Position zeroPos) {
        this.zeroPos = zeroPos;
    }

    @Override
    protected void update() {
        KLog.d("ResetOdometryToPosition", "MANUAL CORNER ZERO, Resetting odometry WheelIMU, " + SharedData.getOdometryWheelIMUPosition() +
                "Odometry Wheel, " + SharedData.getOdometryWheelPosition() +
                " to position, " + zeroPos);
        SharedData.setOdometryWheelPosition(zeroPos);
        SharedData.setOdometryWheelIMUPosition(zeroPos);
        isDone = true;
    }
}
