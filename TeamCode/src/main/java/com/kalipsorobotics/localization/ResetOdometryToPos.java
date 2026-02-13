package com.kalipsorobotics.localization;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;

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
