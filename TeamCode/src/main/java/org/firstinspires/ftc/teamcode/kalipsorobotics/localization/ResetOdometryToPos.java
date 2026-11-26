package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Snaps odometry to zeroPos. The odometry thread reads the pose, adds its delta and writes it back, so a reset written
 * in between is lost. Next update checks the pose landed near zeroPos and writes it again if not (a few tries, so a
 * robot driven away right after the press is never dragged back).
 */
public class ResetOdometryToPos extends Action {
    static final double LANDED_TOLERANCE_MM = 100; // robot sits still for a corner zero; the thread only adds tiny deltas
    static final int MAX_TRIES = 3;

    Position zeroPos;
    private int tries = 0;

    public ResetOdometryToPos(Position zeroPos) {
        this.zeroPos = zeroPos;
    }

    @Override
    protected void update() {
        Position now = SharedData.getOdometryWheelIMUPosition();
        if (tries > 0 && landed(now, zeroPos)) {
            KLog.d("ResetOdometryToPosition", () -> "event=landed pose=" + now + " tries=" + tries);
            isDone = true;
            return;
        }
        if (tries >= MAX_TRIES) {
            KLog.e("ResetOdometryToPosition", "event=gaveUp pose=" + now + " target=" + zeroPos);
            isDone = true;
            return;
        }
        tries++;
        KLog.d("ResetOdometryToPosition", () -> "MANUAL CORNER ZERO try " + tries + ", Resetting odometry WheelIMU, " + now +
                " Odometry Wheel, " + SharedData.getOdometryWheelPosition() + " to position, " + zeroPos);
        SharedData.setOdometryWheelPosition(zeroPos);
        SharedData.setOdometryWheelIMUPosition(zeroPos);
    }

    static boolean landed(Position pose, Position target) {
        return Math.hypot(pose.getX() - target.getX(), pose.getY() - target.getY()) < LANDED_TOLERANCE_MM;
    }
}
