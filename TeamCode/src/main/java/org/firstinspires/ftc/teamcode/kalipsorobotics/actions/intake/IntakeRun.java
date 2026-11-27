package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Pattern: ShooterRun, RPS only (no hood). Holds both intake sides closed loop at a target RPS, either a fixed one
 * (RUN_USING_TARGET_RPS) or one predicted from the distance to targetPoint (RUN_USING_CURRENT_POINT, via
 * Intake.getIntakeRPSPrediction). Done only once stopped (stop() / STOP mode); isWithinRange() says when either side
 * is at speed.
 */
public class IntakeRun extends Action {
    public static final String TAG = "Shooting_IntakeRun";

    private final Intake intake;
    private IntakeRunMode intakeRunMode;
    private Point targetPoint;

    private double targetRPS = 0;
    private double distanceMM = -1;
    private boolean isWithinRange = false;

    /** Direct RPS mode. */
    public IntakeRun(Intake intake, double targetRPS) {
        this.intake = intake;
        this.intakeRunMode = IntakeRunMode.RUN_USING_TARGET_RPS;
        this.targetPoint = null;
        this.targetRPS = targetRPS;
        this.name = "IntakeRun";
    }

    /** RPS from the distance between the current position and targetPoint, every update. */
    public IntakeRun(Intake intake, Point targetPoint) {
        this.intake = intake;
        this.intakeRunMode = IntakeRunMode.RUN_USING_CURRENT_POINT;
        this.targetPoint = targetPoint;
        this.name = "IntakeRun";
    }

    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
            KLog.d(TAG, () -> "event=start mode=" + intakeRunMode + " targetRps=" + targetRPS + " targetPoint=" + targetPoint);
        }

        switch (intakeRunMode) {
            case STOP:
                intake.stop();
                targetRPS = 0;
                isWithinRange = false;
                isDone = true;
                return;
            case RUN_USING_TARGET_RPS:
                distanceMM = -1;
                break;
            case RUN_USING_CURRENT_POINT:
                double currentDist = getCurrentDistanceMM();
                distanceMM = currentDist;
                if (currentDist == -1) {
                    KLog.d(TAG, "event=noDistance keepingTargetRps=" + targetRPS);
                } else {
                    targetRPS = intake.getIntakeRPSPrediction(currentDist);
                }
                break;
        }

        intake.runRPS(targetRPS);
        isWithinRange = intake.getModule(IntakeSide.FRONT).isAtRPS(targetRPS, IntakeConfig.FEED_RPS_TOLERANCE)
                || intake.getModule(IntakeSide.BACK).isAtRPS(targetRPS, IntakeConfig.FEED_RPS_TOLERANCE);

        if (KLog.isDebug()) {
            KLog.d(TAG, () -> "mode=" + intakeRunMode + " targetRps=" + targetRPS + " distanceMm=" + distanceMM
                    + " withinRange=" + isWithinRange);
        }
    }

    // -1 = no target point: RUN_USING_CURRENT_POINT then keeps the RPS it already has.
    private double getCurrentDistanceMM() {
        if (targetPoint == null) {
            return -1;
        }
        return ShooterRun.getDistanceToTargetFromCurrentPos(targetPoint);
    }

    public boolean isWithinRange() {
        return isWithinRange;
    }

    public void stop() {
        setIntakeRunMode(IntakeRunMode.STOP);
        KLog.d(TAG, "event=stop");
    }

    public void setIntakeRunMode(IntakeRunMode intakeRunMode) {
        this.intakeRunMode = intakeRunMode;
        KLog.d(TAG, () -> "event=setMode mode=" + intakeRunMode);
        this.update();
    }

    public IntakeRunMode getIntakeRunMode() {
        return intakeRunMode;
    }

    public void setTargetPoint(Point targetPoint) {
        this.targetPoint = targetPoint;
    }

    public void setTargetRPS(double targetRPS) {
        this.targetRPS = targetRPS;
    }

    public double getTargetRPS() {
        return targetRPS;
    }

    public double getDistanceMM() {
        return distanceMM;
    }

    public Intake getIntake() {
        return intake;
    }
}
