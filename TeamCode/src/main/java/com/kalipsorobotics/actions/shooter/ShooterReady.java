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
    // For direct RPS mode
    private final double targetRPS;
    private double distanceMM = -1;
    private final double targetHoodPosition;

    ElapsedTime elapsedTime;
    final private double timeOutMS;


    public ShooterReady(Shooter shooter, Point target, double distanceMM, double targetRPS, double targetHoodPosition, double timeOutMS) {
        this.shooter = shooter;
        this.target = target;
        this.targetRPS = targetRPS;
        this.targetHoodPosition = targetHoodPosition;
        this.name = "ShooterReady";
        elapsedTime = new ElapsedTime();
        this.timeOutMS = timeOutMS;
        this.distanceMM = distanceMM;
    }

    /**
     * Use For Warmup
     */
    public ShooterReady(Shooter shooter, Point target, LaunchPosition launchPosition, double targetRPS, double targetHoodPosition, double timeOutMS) {
        this(shooter, target, launchPosition.getDistanceToTargetMM(), targetRPS, targetHoodPosition, timeOutMS);
    }
    /**
     * Use For Auto Warmup, and ready
     */
    public ShooterReady(Shooter shooter, Point target, LaunchPosition launchPosition) {
        this(shooter, target, launchPosition, 0, 0, 0);
    }

    /**
     * Use For Maintain
     */
    public ShooterReady(Shooter shooter, Point target, LaunchPosition launchPosition, double timeOutMS) {
        this(shooter, target, launchPosition, 0, 0, timeOutMS);
    }

    /**
     * Use For Data Collection
     */
    public ShooterReady(Shooter shooter, double targetRPS, double targetHoodPosition) {
        this(shooter, null, null, targetRPS, targetHoodPosition, 0);
    }

    public ShooterReady(Shooter shooter, Point target, Point launchPos, double timeOutMS) {
        this(shooter, target, launchPos.distanceTo(target), 0, 0, timeOutMS);
    }
    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        if (!hasStarted) {
            rpsInRangeTimer = new ElapsedTime();
            hasStarted = true;
            rampUpTimeTimer = new ElapsedTime();
            KLog.d(this.getClass().getName(), "PID: " + shooter.getShooter1().getPIDController());
            elapsedTime.reset();
        }



        double rps;
        double hoodPosition;

        // Check if we're in direct RPS mode or auto-prediction mode
        if ((targetRPS > 0) && (targetHoodPosition > 0)) {
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

        if (timeOutMS > 0) {
            if (elapsedTime.milliseconds() > timeOutMS) {
                KLog.d("shooter_ready", "ShooterReady timed out");
                isDone = true;
                return;
            }
        } else {
            if (shooter.isAtTargetRPS()) {
                if ((rpsInRangeTimer.milliseconds() > ShooterConfig.timeToStabilize)) {
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
    }

    private IShooterPredictor.ShooterParams getPrediction() {
        if (distanceMM < 0) {
            KLog.d("shooter_ready", "We using odo pos to target position: " + target);
            // Calculate distance from odometry position to target
            Position currentPosition = SharedData.getOdometryPosition();
            double dx = target.getX() - currentPosition.getX();
            double dy = target.getY() - currentPosition.getY();

//            if (!useAprilTag) {
                distanceMM = Math.sqrt((dx * dx) + (dy * dy));
//            } else {
//                distanceMM = SharedData.getDistanceToGoal();
//            }

        }

        IShooterPredictor.ShooterParams params = shooter.getPrediction(distanceMM);
        KLog.d("shooter_ready", "Distance: " + distanceMM + " params: " + params);
        // Get shooter parameters from predictor using the distance
        return shooter.getPrediction(distanceMM);
    }
}
