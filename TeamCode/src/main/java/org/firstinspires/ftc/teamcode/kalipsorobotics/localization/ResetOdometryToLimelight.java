package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/** Re-zero odometry x/y by the Limelight offset (the window-averaged one AprilTagDetectionAction publishes). */
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
        // Heading stays from the IMU; the fix only corrects x and y. The offset goes onto the pose NOW (the fix can be
        // up to MAX_FIX_AGE_MS old, and the robot may have moved since).
        Position odometry = SharedData.getOdometryWheelIMUPosition();
        double[] offset = SharedData.getLimelightOffset();
        Position limelightGlobalPosition = new Position(odometry.getX() + offset[0], odometry.getY() + offset[1], odometry.getTheta());
        KLog.d(TAG, () -> String.format(java.util.Locale.US, "event=reset from=%s to=%s offX=%.0f offY=%.0f fixAgeMs=%.0f",
                odometry, limelightGlobalPosition, offset[0], offset[1], fixAgeMs));
        SharedData.setOdometryWheelPosition(limelightGlobalPosition);
        SharedData.setOdometryWheelIMUPosition(limelightGlobalPosition);
        SharedData.noteOdometryJump();
        SharedData.clearLimelightFix(); // the old offset is now applied; the next rezero waits for fresh frames
        isDone = true;
    }
}
