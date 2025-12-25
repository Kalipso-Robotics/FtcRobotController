package com.kalipsorobotics.actions.shooter;

import android.annotation.SuppressLint;

import com.kalipsorobotics.modules.shooter.ShooterConfig;
import com.kalipsorobotics.modules.shooter.ShooterRunMode;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.IShooterPredictor;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterRun extends Action {
    private ShooterRunMode shooterRunMode = ShooterRunMode.SHOOT_USING_CURRENT_POINT;
    private final Shooter shooter;
    private final Point targetPoint;
    private Point launchPoint;
    private ElapsedTime rpsInRangeTimer;
    private ElapsedTime rampUpTimeTimer;


    // For direct RPS mode
    private double targetRPS = 0;
    private double targetHoodPosition = 0;
    private double distanceMM = -1;

    ElapsedTime elapsedTime;
    private boolean useLimelight = false;


    public ShooterRun(Shooter shooter, double targetRPS, double targetHoodPosition) {
        this.shooterRunMode = ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD;
        this.shooter = shooter;
        this.targetPoint = null;
        this.name = "ShooterReady";
        elapsedTime = new ElapsedTime();
        this.distanceMM = -1;
        this.targetRPS = targetRPS;
        this.targetHoodPosition = targetHoodPosition;
    }

    public ShooterRun(Shooter shooter, Point targetPoint, Point launchPoint) {
        this.shooterRunMode = ShooterRunMode.SHOOT_USING_FIXED_POINT;
        this.shooter = shooter;
        this.targetPoint = targetPoint;
        this.name = "ShooterReady";
        elapsedTime = new ElapsedTime();
        this.launchPoint = launchPoint;
        this.distanceMM = 0;
        this.targetRPS = 0;
        this.targetHoodPosition = 0;
    }

    public ShooterRun(Shooter shooter, Point targetPoint) {
        this.shooterRunMode = ShooterRunMode.SHOOT_USING_CURRENT_POINT;
        this.shooter = shooter;
        this.targetPoint = targetPoint;
        this.name = "ShooterReady";
        elapsedTime = new ElapsedTime();
        this.distanceMM = -1;
    }

    public boolean isWithinRange() {
        boolean atTarget = shooter.isAtTargetRPS();
        KLog.d("ShooterRun", "isWithinRange called - Current RPS: " + shooter.getRPS() +
               ", At Target: " + atTarget + ", Target RPS: " + targetRPS);
        return atTarget;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        KLog.d("ShooterRun", "update() called - hasStarted: " + hasStarted + ", Current RPS: " + shooter.getRPS());

        if (!hasStarted) {
            shooter.getShooter1().resetPID();
            shooter.getShooter2().resetPID();

            rpsInRangeTimer = new ElapsedTime();
            hasStarted = true;
            rampUpTimeTimer = new ElapsedTime();
            KLog.d("ShooterRun", "*** SHOOTER RUN STARTED ***");
            KLog.d("ShooterRun", "Target RPS: " + targetRPS + ", Target Hood: " + targetHoodPosition + ", Distance: " + distanceMM);
            KLog.d(this.getClass().getName(), "PIDF: " + shooter.getShooter1().getPIDFController());
            elapsedTime.reset();
        }

        KLog.d("ShooterRun", "update() called - hasStarted: " + hasStarted + ", isDone: " + isDone);

        IShooterPredictor.ShooterParams params;

        switch (shooterRunMode) {
            case STOP:
                shooter.stop();
                return;
            case SHOOT_USING_TARGET_RPS_HOOD:
                // Direct RPS mode
                KLog.d("ShooterRun_SHOOT_USING_TARGET_RPS_HOOD", "Using direct RPS mode");
                break;
            case SHOOT_USING_DISTANCE:
                KLog.e("ShooterRun_SHOOT_USING_DISTANCE", "Distance: " + distanceMM);
                params = getPrediction(distanceMM);
                this.targetRPS = params.rps;
                this.targetHoodPosition = params.hoodPosition;
                break;
            case SHOOT_USING_FIXED_POINT:
                if (launchPoint == null) {
                    KLog.e("ShooterRun_SHOOT_USING_FIXED_POINT", "Launch Point Null");
                }
                KLog.d("ShooterRun_SHOOT_USING_FIXED_POINT", "Launch Point " + launchPoint);
                distanceMM = targetPoint.distanceTo(launchPoint);
                params = getPrediction(distanceMM);
                this.targetRPS = params.rps;
                this.targetHoodPosition = params.hoodPosition;
                break;
            case SHOOT_USING_CURRENT_POINT:
                params = getPrediction(getCurrentDistanceMM());
                this.targetRPS = params.rps;
                this.targetHoodPosition = params.hoodPosition;
                KLog.d("ShooterRun_SHOOT_USING_CURRENT_POINT", "TargetRPS: " + targetRPS);
                break;
        }
        KLog.d("ShooterRun", "Running mode " + shooterRunMode);

        // Update hood position
        shooter.getHood().setPosition(targetHoodPosition);

        // Use KMotor's goToRPS for automatic PID control
        shooter.goToRPS(targetRPS);

        // Log status
        KLog.d("shooter_ready", String.format(
            "Target: %.2f RPS, Hood: %.3f",
            targetRPS, targetHoodPosition
        ));


        if (shooter.isAtTargetRPS()) {
            if ((rpsInRangeTimer.milliseconds() > ShooterConfig.timeToStabilize)) {
                KLog.d("shooterAdjust", "Shooter READY " + shooter.getRPS());
                KLog.d("shooter_ready", "ramp up time ms: " + rampUpTimeTimer.milliseconds());
                //isDone = true;
            } else {
                KLog.d("shooter_ready", "waiting for timer, RPS within tolerance: " + shooter.getRPS() + " TARGET: " + targetRPS);
                KLog.d("shooterAdjust", "waiting for timer, RPS within tolerance: " + shooter.getRPS() + " TARGET: " + targetRPS);
            }
        } else {
            KLog.d("shooterAdjust", "Shooter ready timer reset " + shooter.getRPS() + " TARGET: " + targetRPS);
            rpsInRangeTimer.reset();
        }

    }

    private IShooterPredictor.ShooterParams getPrediction(double currentDistanceMM) {
        KLog.d("shooter_ready", "We using odo pos to target position: " + targetPoint);
        // Calculate distance from odometry position to target

        IShooterPredictor.ShooterParams params = shooter.getPrediction(currentDistanceMM);
        KLog.d("shooter_ready", "Distance: " + currentDistanceMM + " params: " + params);
        // Get shooter parameters from predictor using the distance
        return shooter.getPrediction(currentDistanceMM);
    }

    private double getOdometryDistanceMM() {
        double odometryDistanceMM = getDistanceToTargetFromCurrentPos(targetPoint);
        KLog.d("ShooterRun_Distance", "Odometry distance: " + odometryDistanceMM + " mm");
        return odometryDistanceMM;
    }

    private double getCurrentDistanceMM() {
        double distanceMM = getOdometryDistanceMM();
        KLog.d("ShooterRun_Distance", "Odometry Distance " + distanceMM);
        return distanceMM;
    }

    public void setLaunchPoint(Point launchPoint) {
        this.launchPoint = launchPoint;
    }

    public double getTargetRPS() {
        return targetRPS;
    }

    public double getTargetHoodPosition() {
        return targetHoodPosition;
    }

    public double getDistanceMM() {
        return distanceMM;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public void stop() {
        setShooterRunMode(ShooterRunMode.STOP);
        shooter.stop();
        KLog.d("ShooterRun_Stop", "ShooterRun Stop");
    }

    public void setUseLimelight(boolean useLimelight){
        this.useLimelight = useLimelight;
    }

    public void setTargetHoodPosition(double targetHoodPosition) {
        this.targetHoodPosition = targetHoodPosition;
    }

    public void setShooterRunMode(ShooterRunMode shooterRunMode) {
        this.shooterRunMode = shooterRunMode;
    }

    public ShooterRunMode getShooterRunMode() {
        return shooterRunMode;
    }


    public void setTargetRPS(double targetRPS) {
        this.targetRPS = targetRPS;
    }

    public static double getDistanceToTargetFromCurrentPos(Point targetPoint) {
        return SharedData.getOdometryWheelIMUPosition().toPoint().distanceTo(targetPoint);
    }
}
