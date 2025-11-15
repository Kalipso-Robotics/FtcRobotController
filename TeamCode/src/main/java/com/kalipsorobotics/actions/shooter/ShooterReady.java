package com.kalipsorobotics.actions.shooter;

import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.shooter.ShooterConfig;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.IShooterPredictor;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterReady extends Action {

    private final Shooter shooter;
    private final Point target;

    private ElapsedTime rpsInRangeTimer;
    private ElapsedTime rampUpTimeTimer;
    private final LaunchPosition launchPosition;

    // For direct RPS mode
    private final Double targetRPS;
    private final Double targetHoodPosition;

    // Constructor with auto-prediction (existing behavior)
    public ShooterReady(Shooter shooter, Point target, LaunchPosition launchPosition) {
        this.shooter = shooter;
        this.target = target;
        this.launchPosition = launchPosition;
        this.targetRPS = null;
        this.targetHoodPosition = null;
        this.name = "ShooterReady";
    }

    // Constructor with direct RPS and hood position
    public ShooterReady(Shooter shooter, double targetRPS, double targetHoodPosition) {
        this.shooter = shooter;
        this.target = null;
        this.launchPosition = null;
        this.targetRPS = targetRPS;
        this.targetHoodPosition = targetHoodPosition;
        this.name = "ShooterReady";
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        if (!hasStarted) {
            rpsInRangeTimer = new ElapsedTime();
            hasStarted = true;
            rampUpTimeTimer = new ElapsedTime();
            KLog.d(this.getClass().getName(), "PID: " + shooter.getShooter1().getPIDController());
        }

        double rps;
        double hoodPosition;

        // Check if we're in direct RPS mode or auto-prediction mode
        if (targetRPS != null && targetHoodPosition != null) {
            // Direct RPS mode
            rps = targetRPS;
            hoodPosition = targetHoodPosition;
            KLog.d("shooter_ready", "Using direct RPS mode");
        } else {
            // Auto-prediction mode (existing behavior)
            IShooterPredictor.ShooterParams params = getPrediction();
            rps = params.rps;
            hoodPosition = params.hoodPosition;
        }

        // Update hood position
        shooter.getHood().setPosition(hoodPosition);

        // Use KMotor's goToRPS for automatic PID control
        shooter.goToRPS(rps);

        // Log status
        KLog.d("shooter_ready", String.format(
            "Target: %.2f RPS, Hood: %.3f",
            rps, hoodPosition
        ));

        // Check if we've reached target RPS
        if (shooter.isAtTargetRPS()) {
            if (rpsInRangeTimer.milliseconds() > ShooterConfig.timeToStabilize) {
                KLog.d("shooterAdjust", "Shooter READY " + shooter.getRPS());
                KLog.d("shooter_ready", "ramp up time ms: " + rampUpTimeTimer.milliseconds());
                isDone = true;
            } else {
                KLog.d("shooter_ready", "waiting for timer, RPS within tolerance: " + shooter.getRPS() + " TARGET: " + rps);
                KLog.d("shooterAdjust", "waiting for timer, RPS within tolerance: " + shooter.getRPS() + " TARGET: " + rps);
            }
        } else {
            KLog.d("shooterAdjust", "Shooter ready timer reset " + shooter.getRPS() + " TARGET: " + rps);
            rpsInRangeTimer.reset();
        }
    }

    private IShooterPredictor.ShooterParams getPrediction() {
        double distanceMM;

        if (launchPosition == LaunchPosition.AUTO) {
            KLog.d("shooter_ready", "Using target position: " + target);
            // Calculate distance from odometry position to target
            Position currentPosition = SharedData.getOdometryPosition();
            double dx = target.getX() - currentPosition.getX();
            double dy = target.getY() - currentPosition.getY();

//            if (!useAprilTag) {
                distanceMM = Math.sqrt((dx * dx) + (dy * dy));
//            } else {
//                distanceMM = SharedData.getDistanceToGoal();
//            }

        } else {
            KLog.d("shooter_ready", "Using launch position: " + launchPosition);
            // Use the distance from the launch position enum
            distanceMM = launchPosition.getDistanceToTargetMM();
        }
        IShooterPredictor.ShooterParams params = shooter.getPrediction(distanceMM);
        KLog.d("shooter_ready", "Distance: " + distanceMM + " params: " + params);
        // Get shooter parameters from predictor using the distance
        return shooter.getPrediction(distanceMM);
    }
}
