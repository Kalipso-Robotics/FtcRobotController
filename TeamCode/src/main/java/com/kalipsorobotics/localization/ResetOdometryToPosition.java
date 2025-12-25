package com.kalipsorobotics.localization;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToPosition extends Action {
    @Override
    protected void update() {
        if (SharedData.getLimelightRawPosition().isEmpty()) {
            KLog.d("ResetOdometryToPosition", "No Valid Detection Not Updating");
            return;
        }
        Position limelightGlobalPosition = SharedData.getLimelightGlobalPosition();
        KLog.d("ResetOdometryToPosition", "Resetting odometry WheelIMU, " + SharedData.getOdometryWheelIMUPosition() + "Odometry Wheel, " + SharedData.getOdometryWheelPosition() + " to position, " + limelightGlobalPosition);
        SharedData.setOdometryWheelPosition(limelightGlobalPosition);
        SharedData.setOdometryWheelIMUPosition(limelightGlobalPosition);
        isDone = true;
    }
}
