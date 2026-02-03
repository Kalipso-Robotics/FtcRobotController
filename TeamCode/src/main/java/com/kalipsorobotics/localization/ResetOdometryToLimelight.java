package com.kalipsorobotics.localization;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToLimelight extends Action {

    Turret turret;

    public ResetOdometryToLimelight(Turret turret) {
        this.turret = turret;
    }


    @Override
    protected void update() {
        double turretVelocity = turret.getCurrentVelocity();
        if (SharedData.getLimelightRawPosition().isEmpty() && turretVelocity > 10) {
            KLog.d("ResetOdometryToPosition", "No Valid Detection Not Updating. turretVelocity (deg / sec):  " + turretVelocity);
            return;
        }
        Position limelightGlobalPosition = SharedData.getLimelightGlobalPosition();
        KLog.d("ResetOdometryToPosition", "If I were resetting but I am not. Resetting odometry WheelIMU, " + SharedData.getOdometryWheelIMUPosition() + "Odometry Wheel, " + SharedData.getOdometryWheelPosition() + " to position, " + limelightGlobalPosition);
        SharedData.setOdometryWheelPosition(limelightGlobalPosition);
        SharedData.setOdometryWheelIMUPosition(limelightGlobalPosition);
        isDone = true;
    }
}
