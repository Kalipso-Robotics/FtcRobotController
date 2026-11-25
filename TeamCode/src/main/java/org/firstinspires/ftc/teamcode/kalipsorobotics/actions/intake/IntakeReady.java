package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Shot gate for the transfer, like ShooterReady / TurretReady: holds both sides at IntakeConfig.FEED_RPS (closed loop,
 * stopper still closed) and is done once both are within IntakeConfig.FEED_RPS_TOLERANCE. Never blocks forever: with
 * balls against the closed stopper the transfer may not reach FEED_RPS, so after FEED_GATE_TIMEOUT_MS it is done with
 * timedOut() = true (logged). FEED_RPS <= 0 (closed loop off) = done at once. Once done the caller keeps the hold
 * (holdFeedRPS) until PushBall takes the motors.
 */
public class IntakeReady extends Action {
    public static final String TAG = "Shooting_TransferReady";

    private final IntakeSide sideA;
    private final IntakeSide sideB;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean timedOut = false;

    public IntakeReady(Intake intake) {
        sideA = intake.getSide(BeamBreakPosition.IntakeSide.A);
        sideB = intake.getSide(BeamBreakPosition.IntakeSide.B);
    }

    public boolean timedOut() {
        return timedOut;
    }

    /** Both sides closed loop at IntakeConfig.FEED_RPS (no-op when it is <= 0). */
    public void holdFeedRPS() {
        if (IntakeConfig.FEED_RPS > 0) {
            sideA.runRPS(IntakeConfig.FEED_RPS);
            sideB.runRPS(IntakeConfig.FEED_RPS);
        }
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            hasStarted = true;
            timer.reset();
            KLog.d(TAG, () -> "event=start targetRps=" + IntakeConfig.FEED_RPS + " tolRps=" + IntakeConfig.FEED_RPS_TOLERANCE);
        }
        if (isDone) {
            return;
        }
        double target = IntakeConfig.FEED_RPS;
        if (target <= 0) {
            isDone = true;
            KLog.d(TAG, "event=skip reason=closedLoopOff");
            return;
        }
        // RPS read before this loop's command: the result of the last one
        boolean a = sideA.isAtRPS(target, IntakeConfig.FEED_RPS_TOLERANCE);
        boolean b = sideB.isAtRPS(target, IntakeConfig.FEED_RPS_TOLERANCE);
        holdFeedRPS();
        if (a || b) {
            isDone = true;
            KLog.d(TAG, () -> String.format("event=ready waitMs=%.0f rpsA=%.2f rpsB=%.2f", timer.milliseconds(), sideA.getRPS(), sideB.getRPS()));
        } else if (timer.milliseconds() >= IntakeConfig.FEED_GATE_TIMEOUT_MS) {
            isDone = true;
            timedOut = true;
            KLog.e(TAG, String.format("event=timeout waitMs=%.0f rpsA=%.2f rpsB=%.2f inTolA=%b inTolB=%b",
                    timer.milliseconds(), sideA.getRPS(), sideB.getRPS(), a, b));
        }
    }
}
