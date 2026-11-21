package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToLimelight extends Action {
    public static final String TAG = "Vision_OdoReset";

    Turret turret;

    public ResetOdometryToLimelight(Turret turret) {
        this.turret = turret;
    }

    @Override
    protected void update() {
        double fixAgeMs = SharedData.getLimelightFixAgeMs();
        if (fixAgeMs > AprilTagConfig.MAX_FIX_AGE_MS) {
            KLog.d(TAG, () -> "event=skip reason=noFreshFix ageMs=" + fixAgeMs);
            isDone = true;
            return;
        }
        // No turret encoder: while the servo is still travelling, the commanded angle is not where the camera points.
        if (turret.isMoving()) {
            KLog.d(TAG, "event=skip reason=turretMoving");
            isDone = true;
            return;
        }
        // Heading stays from the IMU; the fix only corrects x and y.
        Position odometry = SharedData.getOdometryWheelIMUPosition();
        Position fix = SharedData.getLimelightGlobalPosition();
        Position limelightGlobalPosition = new Position(fix.getX(), fix.getY(), odometry.getTheta());
        KLog.d(TAG, () -> "event=reset from=" + SharedData.getOdometryWheelIMUPosition() + " to=" + limelightGlobalPosition);
        SharedData.setOdometryWheelPosition(limelightGlobalPosition);
        SharedData.setOdometryWheelIMUPosition(limelightGlobalPosition);
        isDone = true;
    }
}
