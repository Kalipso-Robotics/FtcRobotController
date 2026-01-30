package com.kalipsorobotics.actions.shooter;

import static com.kalipsorobotics.decode.configs.ShooterInterpolationConfig.*;

import android.annotation.SuppressLint;

import com.kalipsorobotics.decode.configs.AprilTagConfig;
import com.kalipsorobotics.decode.configs.ShooterConfig;
import com.kalipsorobotics.modules.shooter.ShooterRunMode;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.shooter.IShooterPredictor;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterRun extends Action {

    public static final double FAR_TOLERANCE = 0.5;
    public static final double MIDDLE_TOLERANCE = 0.5;
    public static final double NEAR_TOLERANCE = 0.5;
    private final OpModeUtilities opModeUtilities;
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

    private boolean useOdometry = true;
    private boolean isWithinRange = false;



    public ShooterRun(OpModeUtilities opModeUtilities, Shooter shooter, double targetRPS, double targetHoodPosition) {
        this.opModeUtilities = opModeUtilities;
        this.shooterRunMode = ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD;
        this.shooter = shooter;
        this.targetPoint = null;
        this.name = "ShooterReady";
        elapsedTime = new ElapsedTime();
        this.distanceMM = -1;
        this.targetRPS = targetRPS;
        this.targetHoodPosition = targetHoodPosition;
    }

    public ShooterRun(OpModeUtilities opModeUtilities, Shooter shooter, Point targetPoint, Point launchPoint) {
        this.opModeUtilities = opModeUtilities;
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

    public ShooterRun(OpModeUtilities opModeUtilities, Shooter shooter, Point targetPoint) {
        this.opModeUtilities = opModeUtilities;
        this.shooterRunMode = ShooterRunMode.SHOOT_USING_CURRENT_POINT;
        this.shooter = shooter;
        this.targetPoint = targetPoint;
        this.name = "ShooterReady";
        elapsedTime = new ElapsedTime();
        this.distanceMM = -1;
    }

    public boolean isWithinRange() {
        //boolean atTarget = shooter.isAtTargetRPS();
        KLog.d("ShooterRun", "isWithinRange called. Power: " + shooter.getShooter1().getPower() + " Current RPS: " + shooter.getRPS() +
                ", Target RPS: " + targetRPS + ", At Within Range: " + isWithinRange);
        return isWithinRange;
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
                KLog.d("ShooterRun_SHOOT_USING_TARGET_RPS_HOOD", "Using direct RPS mode, TargetRps " + targetRPS + " Target Hood " + targetHoodPosition);
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
                double currentDist = getCurrentDistanceMM();
                if (currentDist == -1) {
                    KLog.d("ShooterRun_SHOOT_USING_CURRENT_POINT", "NO DISTANCE FOUND, NOT CHANGING RPS OR HOOD");
                } else {
                    params = getPrediction(currentDist);
                    this.targetRPS = params.rps;
                    this.targetHoodPosition = params.hoodPosition;
                    KLog.d("ShooterRun_SHOOT_USING_CURRENT_POINT", "TargetRPS: " + targetRPS);
                }
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


        if (shooter.isAtTargetRPS(calculateRPSTolerance())) {
            if ((rpsInRangeTimer.milliseconds() > ShooterConfig.timeToStabilize)) {
                KLog.d("shooter_ready", "Shooter is within range and READY. TargetRPS: " + targetRPS + " Current RPS: " + shooter.getRPS() + ". Ramp up time ms:" + rampUpTimeTimer.milliseconds());
                //isDone = true;
                isWithinRange = true;
            } else {
                KLog.d("shooter_ready", "Shooter within range, but waiting for time to stablize. TargetRPS: " + targetRPS + " Current RPS: " + shooter.getRPS() );
                isWithinRange = false;
            }
        } else {
            KLog.d("shooter_ready", "Shooter not within range." + shooter.getRPS() + " TARGET: " + targetRPS);
            rpsInRangeTimer.reset();
            isWithinRange = false;
        }

    }

    private IShooterPredictor.ShooterParams getPrediction(double currentDistanceMM) {
        KLog.d("shooter_ready", "We using odo pos to target position: " + targetPoint);
        // Calculate distance from odometry position to target

        IShooterPredictor.ShooterParams params = shooter.getPrediction(currentDistanceMM);
        KLog.d("shooter_ready", "Distance: " + currentDistanceMM + " params: " + params);
        // Get shooter parameters from predictor using the distance
        return params;
    }

    private double getOdometryDistanceMM() {
        double odometryDistanceMM = getDistanceToTargetFromCurrentPos(targetPoint) - AprilTagConfig.GOAL_TO_APRIL_TAG_OFFSET_DISTANCE;
        KLog.d("ShooterRun_Distance", "Odometry distance: " + odometryDistanceMM + " mm");
        return odometryDistanceMM;
    }

    private double getLimelightDistance() {
        double limelightDistanceMM = SharedData.getLimelightRawPosition().getAprilTagDistanceToCamMM();
        KLog.d("ShooterRun_Distance", "Limelight distance: " + limelightDistanceMM + " mm");
        return limelightDistanceMM;
    }

    private double getCurrentDistanceMM() {
        double distanceMM = -1;
        distanceMM = getOdometryDistanceMM();
        KLog.d("ShooterRun_Distance", "ODOMETRY DISTANCE " + distanceMM + " mm");
        if (useLimelight && !SharedData.getLimelightRawPosition().isEmpty()) {
            distanceMM = getLimelightDistance();
            KLog.d("ShooterRun_Distance", "OVERRIDING ODOMETRY DISTANCE WITH LIMELIGHT " + distanceMM + " mm");
        }
        KLog.d("ShooterRun_Distance", "Final Distance " + distanceMM);
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
        KLog.d("ShooterRun_Set_Mode", "Mode Change -> Force Update. New Mode: " + shooterRunMode);
        this.update();

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

    public void setUseOdometry(boolean useOdometry) {
        this.useOdometry = useOdometry;
        KLog.d("ShooterRun", "Odometry was set to: " + useOdometry);
    }


    public double calculateRPSTolerance() {
        if (distanceMM > FAR_DISTANCE) {
            return FAR_TOLERANCE;
        } else if (distanceMM > NEAR_DISTANCE) {
            return MIDDLE_TOLERANCE;
        } else {
            return NEAR_TOLERANCE;
        }
    }
}
