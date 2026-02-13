package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;


public class AdjustShooterSpeedAction extends Action {
    private double targetRPS;
    private final double timeoutMS;
    private ElapsedTime timeoutTimer;
    private final Shooter shooter;
    private static final double DEFAULT_DROP_BELOW_THRESHOLD_PERCENT = 0.99;
    private static final double DEFAULT_TIMEOUT_MS = 1000;
    private static final double MAX_RPS = 51;
    private final double dropBelowThreshold;
//    private boolean boosting = false;
    private final double motorDirection;

    // Ideal constructor: just shooter
    public AdjustShooterSpeedAction(Shooter shooter) {
        this(shooter, DEFAULT_TIMEOUT_MS, DEFAULT_DROP_BELOW_THRESHOLD_PERCENT);
    }

    // Full control constructor
    public AdjustShooterSpeedAction(Shooter shooter, double timeoutMS, double dropBelowThresholdPercent) {
        this.shooter = shooter;
        this.timeoutMS = timeoutMS;
        this.dropBelowThreshold = dropBelowThresholdPercent;
        double currentPower = shooter.getShooter1().getPower();
        this.motorDirection = (currentPower < 0) ? -1 : 1;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        KLog.d("shooterAdjust", "RPS " + shooter.getRPS());

        if (!hasStarted) {
            targetRPS = shooter.getRPS();
            timeoutTimer = new ElapsedTime();
            hasStarted = true;
            KLog.d("shooterAdjust", "shooter boost started, targetRPS " + targetRPS);
        }

        if (shooter.getRPS() < targetRPS * dropBelowThreshold) {
                double boostPower = calculateBoostPower();
                //shooter.setPower(boostPower);
                KLog.d("shooterAdjust", "BOOSTING started, boosted to : " + boostPower);
        }

        if (timeoutTimer.milliseconds() > timeoutMS) {
            KLog.d("shooterAdjust", "shooter boost done");
            isDone = true;
        }

    }

    private double calculateBoostPower() {
        double rpsRatio = targetRPS / MAX_RPS;
        double magnitude = Math.min(1.0, Math.max(0.8, rpsRatio));
        return motorDirection * magnitude;
    }
}


