package com.kalipsorobotics.actions.shooter;

import android.annotation.SuppressLint;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.IShooterPredictor;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.SharedData;

public class ShooterReady extends Action {

    private final Shooter shooter;
    private final Point target;
    private final LaunchPosition launchPosition;

    private static final double TARGET_RPS_TOLERANCE = 1.0;

    public ShooterReady(Shooter shooter, Point target, LaunchPosition launchPosition) {
        this.shooter = shooter;
        this.target = target;
        this.launchPosition = launchPosition;
        this.name = "ShooterReady";
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
        }

        // Get shooter parameters for target RPS and hood position
        IShooterPredictor.ShooterParams params = getPrediction();

        // Update hood position (offset already included in predictor)
        shooter.getHood().setPosition(params.hoodPosition);

        // Use KMotor's goToRPS for automatic PID control
        shooter.goToRPS(params.rps);

        // Get current RPS for logging and checking
//        double currentRPS = shooter.getRPS();
//        double error = params.rps - currentRPS;
//
//        // Log status
        KLog.d("shooter_ready", String.format(
            "Target: %.2f RPS, Hood: %.3f",
            params.rps, params.hoodPosition
        ));

        // Check if we've reached target RPS
        if (shooter.getShooter1().isAtTargetRPS(TARGET_RPS_TOLERANCE)) {
            isDone = true;
            KLog.d("shooter_ready", "RPS within tolerance - Ready!");
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
            distanceMM = Math.sqrt((dx * dx) + (dy * dy));
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
