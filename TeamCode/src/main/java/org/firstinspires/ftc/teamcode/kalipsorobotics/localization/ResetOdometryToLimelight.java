package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToLimelight extends Action {
    public static final String TAG = "Vision_OdoReset";

    Turret turret;

    private boolean overrideOdometry = false;

    public ResetOdometryToLimelight(Turret turret) {
        this.turret = turret;
    }

    @Override
    protected void update() {
        if (SharedData.getLimelightRawPosition().isEmpty()) {
            KLog.d(TAG, "event=skip reason=noDetection");
            isDone = true;
            return;
        }
        // No turret encoder: while the servo is still travelling, the commanded angle is not where the camera points.
        if (turret.isMoving()) {
            KLog.d(TAG, "event=skip reason=turretMoving");
            isDone = true;
            return;
        }
        Position limelightGlobalPosition = SharedData.getLimelightGlobalPosition();
        KLog.d(TAG, () -> "event=reset from=" + SharedData.getOdometryWheelIMUPosition() + " to=" + limelightGlobalPosition);
        SharedData.setOdometryWheelPosition(limelightGlobalPosition);
        SharedData.setOdometryWheelIMUPosition(limelightGlobalPosition);
        isDone = true;
    }

    public void setOverrideOdometry(boolean overrideOdometry) {
        this.overrideOdometry = overrideOdometry;
    }
}
