package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Shot gate for the intake, like ShooterReady: done once IntakeRun reports either side within
 * IntakeConfig.FEED_RPS_TOLERANCE of its target RPS. Never blocks forever: with balls against the closed stopper the
 * intake may not reach speed, so after FEED_GATE_TIMEOUT_MS it is done with timedOut() = true (logged).
 * IntakeRun does the spinning; this only checks it.
 */
public class IntakeReady extends Action {
    public static final String TAG = "Shooting_IntakeReady";

    private final IntakeRun intakeRun;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean timedOut = false;

    public IntakeReady(IntakeRun intakeRun) {
        this.intakeRun = intakeRun;
    }

    public boolean timedOut() {
        return timedOut;
    }

    public IntakeRun getIntakeRun() {
        return intakeRun;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            timer.reset();
            KLog.d(TAG, () -> "event=start targetRps=" + intakeRun.getTargetRPS() + " tolRps=" + IntakeConfig.FEED_RPS_TOLERANCE);
        }

        if (intakeRun.isWithinRange()) {
            isDone = true;
            KLog.d(TAG, () -> String.format("event=ready waitMs=%.0f targetRps=%.2f distanceMm=%.0f",
                    timer.milliseconds(), intakeRun.getTargetRPS(), intakeRun.getDistanceMM()));
        } else if (timer.milliseconds() >= IntakeConfig.FEED_GATE_TIMEOUT_MS) {
            isDone = true;
            timedOut = true;
            KLog.e(TAG, String.format("event=timeout waitMs=%.0f targetRps=%.2f", timer.milliseconds(), intakeRun.getTargetRPS()));
        } else {
            KLog.d(TAG, "event=waiting reason=rpsOutOfRange");
        }
    }
}
