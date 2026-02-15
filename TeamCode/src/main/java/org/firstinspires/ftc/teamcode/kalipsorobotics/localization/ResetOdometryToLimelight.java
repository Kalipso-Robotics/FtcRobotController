package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToLimelight extends Action {

    Turret turret;

    private boolean overrideOdometry = false;

    public ResetOdometryToLimelight(Turret turret) {
        this.turret = turret;
    }

    @Override
    protected void update() {
        double turretVelocity = turret.getCurrentVelocity();
        if (SharedData.getLimelightRawPosition().isEmpty() || turretVelocity > 10) {
            KLog.d("ResetOdometryToPosition", "No Valid Detection Not Updating. turretVelocity (deg / sec):  " + turretVelocity);
            isDone = true;
            return;
        }
        Position limelightGlobalPosition = SharedData.getLimelightGlobalPosition();
        KLog.d("ResetOdometryToPosition", "If I were resetting but I am not. Resetting odometry WheelIMU, " + SharedData.getOdometryWheelIMUPosition() + " Odometry Wheel, " + SharedData.getOdometryWheelPosition() + " to position, " + limelightGlobalPosition);
        SharedData.setOdometryWheelPosition(limelightGlobalPosition);
        SharedData.setOdometryWheelIMUPosition(limelightGlobalPosition);
        isDone = true;
    }

    public void setOverrideOdometry(boolean overrideOdometry) {
        this.overrideOdometry = overrideOdometry;
    }
}
