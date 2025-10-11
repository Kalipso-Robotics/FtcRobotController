package com.kalipsorobotics.actions.shooter;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.modules.shooter.ShooterLutPredictor;
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

    @Override
    public void update() {
        if (!hasStarted) {
            hasStarted = true;
        }

        // Get prediction for target RPS and hood position
        ShooterLutPredictor.Prediction prediction = getPrediction();
        if (prediction == null) {
            KLog.e("ShooterReady", "Failed to get prediction");
            return;
        }

        // Update hood position
        shooter.getHood().setPosition(prediction.hood);

        // Use KMotor's goToRPS for automatic PID control
        shooter.goToRPS(prediction.rps);

        // Get current RPS for logging and checking
        double currentRPS = shooter.getRPS();
        double error = prediction.rps - currentRPS;

        // Log status
        KLog.d("ShooterReady", String.format(
            "Target: %.2f RPS, Current: %.2f RPS, Error: %.2f RPS, Hood: %.3f",
            prediction.rps, currentRPS, error, prediction.hood
        ));

        // Check if we've reached target RPS
        if (shooter.getShooter1().isAtTargetRPS(TARGET_RPS_TOLERANCE)) {
            isDone = true;
            KLog.d("ShooterReady", "RPS within tolerance - Ready!");
        }
    }

    private ShooterLutPredictor.Prediction getPrediction() {
        double distanceMM;

        if (launchPosition == LaunchPosition.AUTO) {
            // Calculate distance from odometry position to target
            Position currentPosition = SharedData.getOdometryPosition();
            double dx = target.getX() - currentPosition.getX();
            double dy = target.getY() - currentPosition.getY();
            distanceMM = Math.sqrt((dx * dx) + (dy * dy));
        } else {
            // Use the distance from the launch position enum
            distanceMM = launchPosition.getDistanceToTargetMM();
        }

        // Get prediction from shooter using the distance
        return shooter.getPrediction(distanceMM);
    }
}
