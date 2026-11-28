package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.shouldBangBangCompensate;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig.*;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.SOTMConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.IShooterPredictor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterRun extends Action {
    public static final String TAG = "Shooting_ShooterRun";

    public static final double FAR_TOLERANCE = 0.75; // TODO-measure
    public static final double MIDDLE_TOLERANCE = 0.75; // TODO-measure
    public static final double NEAR_TOLERANCE = 0.60; // TODO-measure; RPS reads in 0.714 steps (20 ticks/s, 28 CPR)
    private final OpModeUtilities opModeUtilities;
    private ShooterRunMode shooterRunMode = ShooterRunMode.SHOOT_USING_CURRENT_POINT;
    private final Shooter shooter;
    private Point targetPoint;
    private Point launchPoint;
    private ElapsedTime rpsInRangeTimer;
    private ElapsedTime rampUpTimeTimer;


    // For direct RPS mode
    private double targetRPS = 0;
    private double targetHoodPosition = 0;
    private double distanceMM = -1;
    ElapsedTime elapsedTime;

    private boolean useOdometry = true;
    private boolean isWithinRange = false;

    private double currentRPS = 0;

    private DcMotor.ZeroPowerBehavior prevMode;

    public ShooterRun(OpModeUtilities opModeUtilities, Shooter shooter, double targetRPS, double targetHoodPosition) {
        this.opModeUtilities = opModeUtilities;
        this.shooterRunMode = ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD;
        this.shooter = shooter;
        this.targetPoint = null;
        this.name = "ShooterRun";
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
        this.name = "ShooterRun";
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
        this.name = "ShooterRun";
        elapsedTime = new ElapsedTime();
        this.distanceMM = -1;
    }

    public boolean isWithinRange() {
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> "event=isWithinRange power=" + shooter.getShooterMotor().getPower() + " currentRps=" + currentRPS +
                    " targetRps=" + targetRPS + " withinRange=" + isWithinRange);
        }
        return isWithinRange;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        currentRPS = shooter.getRPS();
        if (!hasStarted) {
            shooter.getShooterMotor().resetPID();

            rpsInRangeTimer = new ElapsedTime();
            hasStarted = true;
            rampUpTimeTimer = new ElapsedTime();
            KLog.d(TAG, () -> "event=start targetRps=" + targetRPS + " targetHood=" + targetHoodPosition + " distanceMm=" + distanceMM
                    + " pidf=" + shooter.getShooterMotor().getPIDFController());
            elapsedTime.reset();
        }

        if (KLog.isDebug()) {
            KLog.d(TAG, () -> "currentRps=" + currentRPS + " isDone=" + isDone);
        }

        IShooterPredictor.ShooterParams params;

        switch (shooterRunMode) {
            case STOP:
                shooter.stop();
                // Zero the target so the continuous hold (Shooter.holdTargetRPS) keeps the
                // flywheel stopped instead of spinning it back up to the last shooting target.
                shooter.setTargetRPS(0);
                targetRPS = 0;
                logPowerIfZero();
                return;
            case SHOOT_USING_TARGET_RPS_HOOD:
                // Direct RPS mode
                distanceMM = -1;
                KLog.d("ShooterRun_SHOOT_USING_TARGET_RPS_HOOD", () -> "Using direct RPS mode, TargetRps " + targetRPS + " Target Hood " + targetHoodPosition);
                break;
            case SHOOT_USING_DISTANCE:
                KLog.e("ShooterRun_SHOOT_USING_DISTANCE", "Distance: " + distanceMM);
                params = getPrediction(distanceMM);
                this.targetRPS = params.shooterRPS;
                this.targetHoodPosition = params.hoodPosition;
                break;
            case SHOOT_USING_FIXED_POINT:
                if (launchPoint == null) {
                    KLog.e("ShooterRun_SHOOT_USING_FIXED_POINT", "Launch Point Null");
                }
                KLog.d("ShooterRun_SHOOT_USING_FIXED_POINT", () -> "Launch Point " + launchPoint);
                distanceMM = targetPoint.distanceTo(launchPoint);
                params = getPrediction(distanceMM);
                this.targetRPS = params.shooterRPS;
                this.targetHoodPosition = params.hoodPosition;
                KLog.d("ShooterRun_SHOOT_USING_FIXED_POINT", "TargetRPS: " + targetRPS + " distanceMM: " + distanceMM);
                break;
            case SHOOT_USING_CURRENT_POINT:
                double currentDist = getCurrentDistanceMM();
                distanceMM = currentDist;
                if (currentDist == -1) {
                    KLog.d("ShooterRun_SHOOT_USING_CURRENT_POINT", "NO DISTANCE FOUND, NOT CHANGING RPS OR HOOD");
                } else {
                    params = getPrediction(currentDist);
                    this.targetRPS = params.shooterRPS;
                    this.targetHoodPosition = params.hoodPosition;
                    KLog.d("ShooterRun_SHOOT_USING_CURRENT_POINT", () -> "TargetRPS: " + targetRPS);
                }
                break;
        }
        shooter.setTargetRPS(targetRPS);
        KLog.d("ShooterRun_RunModeData", () -> "Running mode " + shooterRunMode + " targetRPS: " + targetRPS + " distance: " + distanceMM);

        double deltaRPS = targetRPS - currentRPS;
        double hoodCompensation = MathFunctions.clamp(deltaRPS * hoodCompensateCoefficient, minHoodCompensate, maxHoodCompensate);
        double effectiveTargetHood = MathFunctions.clamp(targetHoodPosition + hoodCompensation, MIN_HOOD, MAX_HOOD);

        if (Math.abs(deltaRPS) > 1) {
            KLog.d("ShooterRun_Hood", () -> "================ Big drop: Hood Compensation: " + hoodCompensation + " effectiveTargetHood: " + effectiveTargetHood + " targetHoodPosition: " + targetHoodPosition + " Delta RPS: " + deltaRPS);
        } else {
            KLog.d("ShooterRun_Hood", () -> "Hood Compensation: " + hoodCompensation +
                    " effectiveTargetHood: " + effectiveTargetHood +
                    " targetHoodPosition: " + targetHoodPosition +
                    " Delta RPS: " + deltaRPS +
                    " Current RPS: " + currentRPS +
                    " Target RPS: " + targetRPS
            );
        }

        // Bang control for rapid accel/decel when error is large
        if (deltaRPS > ShooterConfig.accelBoostDeltaRPSThreshold && shouldBangBangCompensate) {
            // Need to ACCELERATE - current RPS too low
            if (prevMode != DcMotor.ZeroPowerBehavior.FLOAT) {
                shooter.getShooterMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                prevMode = DcMotor.ZeroPowerBehavior.FLOAT;
            }
            shooter.setPower(calculateEffectiveCompensationPower());
            KLog.d("ShooterRun_BangControl", () -> "BANG ACCEL: deltaRPS=" + deltaRPS + " (threshold=" + ShooterConfig.accelBoostDeltaRPSThreshold + ")");
        } else if (deltaRPS < ShooterConfig.decelBoostDeltaRPSThreshold && shouldBangBangCompensate) {
            // Need to DECELERATE - current RPS too high
            if (prevMode != DcMotor.ZeroPowerBehavior.BRAKE) {
                shooter.getShooterMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                prevMode = DcMotor.ZeroPowerBehavior.BRAKE;
            }
            shooter.setPower(0);
            KLog.d("ShooterRun_BangControl", () -> String.format("BANG DECEL: deltaRPS = %.2f, threshold = %.2f", deltaRPS, ShooterConfig.decelBoostDeltaRPSThreshold));
        } else {
            // Within threshold - use PID control for fine adjustment
            if (prevMode != DcMotor.ZeroPowerBehavior.FLOAT) {
                shooter.getShooterMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                prevMode = DcMotor.ZeroPowerBehavior.FLOAT;
            }
            shooter.goToRPS(targetRPS);
            KLog.d("ShooterRun_BangControl", () -> String.format("PID CONTROL: deltaRPS = %.2f", deltaRPS));
        }

        // Debug: if the shooter motor power ended up at 0.0, dump the stack trace so we can
        // see which code path drove the power to zero during the routine.
        logPowerIfZero();

        // Update hood position
        shooter.getHood().setPosition(effectiveTargetHood);

        KLog.d("shooter_ready", () -> String.format(
            "Target: %.2f RPS, Hood: %.3f",
            targetRPS, targetHoodPosition
        ));


        if (shooter.isAtTargetRPS(calculateRPSTolerance())) {
            if ((rpsInRangeTimer.milliseconds() > ShooterConfig.timeToStabilize)) {
                KLog.d("shooter_ready", () -> "Shooter is within range and READY. TargetRPS: " + targetRPS + " Current RPS: " + currentRPS + ". Ramp up time ms:" + rampUpTimeTimer.milliseconds());
                isWithinRange = true;
            } else {
                KLog.d("shooter_ready", () -> "Shooter within range, but waiting for time to stabilize. TargetRPS: " + targetRPS + " Current RPS: " + currentRPS);
                isWithinRange = false;
            }
        } else {
            KLog.d("shooter_ready", () -> "Shooter not within range." + currentRPS + " TARGET: " + targetRPS);
            rpsInRangeTimer.reset();
            isWithinRange = false;
        }

    }

    private IShooterPredictor.ShooterParams getPrediction(double currentDistanceMM) {
        KLog.d("shooter_ready", () -> "We using odo pos to target position: " + targetPoint);
        IShooterPredictor.ShooterParams params = shooter.getPrediction(currentDistanceMM);
        KLog.d("shooter_ready", () -> "Distance: " + currentDistanceMM + " params: " + params);
        return params;
    }

    private double getOdometryDistanceMM() {
        // The direct RPS constructor has no target; ShootAllAction still flips to CURRENT_POINT after the feed.
        // -1 = no distance: CURRENT_POINT then holds the RPS and hood it already has.
        if (targetPoint == null) {
            KLog.d("ShooterRun_Distance", "No target point, no distance");
            return -1;
        }
        double odometryDistanceMM = getDistanceToTargetFromCurrentPos(targetPoint);
        KLog.d("ShooterRun_Distance", () -> "Odometry distance: " + odometryDistanceMM + " mm");
        return odometryDistanceMM;
    }

    // Distance always comes from odometry (Limelight re-zeros odometry instead of overriding the distance).
    private double getCurrentDistanceMM() {
        return getOdometryDistanceMM();
    }

    /** The goal used for the distance in SHOOT_USING_CURRENT_POINT; RPS and hood follow it on the next update. */
    public void setTargetPoint(Point targetPoint) {
        this.targetPoint = targetPoint;
    }

    public Point getTargetPoint() {
        return targetPoint;
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

    public void setTargetHoodPosition(double targetHoodPosition) {
        this.targetHoodPosition = targetHoodPosition;
    }

    public void setShooterRunMode(ShooterRunMode shooterRunMode) {
        this.shooterRunMode = shooterRunMode;
        KLog.d("ShooterRun_Set_Mode", () -> "Mode Change -> Force Update. New Mode: " + shooterRunMode);
        this.update();

    }
    public ShooterRunMode getShooterRunMode() {
        return shooterRunMode;
    }


    public void setTargetRPS(double targetRPS) {
        this.targetRPS = targetRPS;
    }

    public static double getDistanceToTargetFromCurrentPos(Point targetPoint) {
        SOTM.Solution sotm = SOTMConfig.adjustShooterRPS ? SOTM.usableSolution() : null;
        if (sotm != null) {
            return sotm.shotDistanceMM;
        }
        // From the turret pivot, like SOTM, so the distance table means the same thing with SOTM on or off.
        return Turret.launcherPoint(SharedData.peekOdometryWheelIMUPosition()).distanceTo(targetPoint);
    }

    public void setUseOdometry(boolean useOdometry) {
        this.useOdometry = useOdometry;
        KLog.d(TAG, () -> "event=setUseOdometry useOdometry=" + useOdometry);
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

    public double calculateEffectiveCompensationPower() {
        if (distanceMM > NEAR_DISTANCE) {
            return ShooterConfig.BANG_POWER_FAR;
        } else {
            return ShooterConfig.BANG_POWER_NEAR;
        }
    }

    public double getCurrentRPS() {
        return currentRPS;
    }

    /**
     * When true, {@link #logPowerIfZero()} dumps a full stack trace every time the shooter power
     * is floored to 0 while a non-zero RPS is still wanted. This is a HEAVY diagnostic - a stack
     * walk plus ~30 logcat writes - and it runs inside the per-loop control path. Leaving it on
     * throttles the main loop and degrades everything that shares that loop (turret auto-align,
     * shooter RPS settling). Keep it false for normal driving/matches; flip it true only for a
     * focused debug session, then turn it back off.
     */
    public static boolean DEBUG_POWER_ZERO_STACK_TRACE = false;

    /**
     * Reads back the actual shooter motor power and, if it is 0.0, logs the full stack trace
     * to the debug log. This is a debugging aid to discover which caller / code path is
     * driving the shooter power to zero during the routine. Gated behind
     * {@link #DEBUG_POWER_ZERO_STACK_TRACE} so it is zero-cost during normal play.
     */
    private void logPowerIfZero() {
        if (!DEBUG_POWER_ZERO_STACK_TRACE) {
            return;
        }

        double power = shooter.getShooterMotor().getPower();

        boolean powerIsZero = Math.abs(power) < 1e-9;
        if (!powerIsZero) {
            return;
        }

        // Power 0 while target is 0 (STOP / park) is expected - just note it, no stack trace.
        if (Math.abs(targetRPS) < 1e-9) {
            KLog.d("ShooterRun_PowerZero", () -> "Power 0.0 with target 0 (expected stop) -> shooter: " +
                    power + " | mode: " + shooterRunMode);
            return;
        }

        // Power 0 while we still want a non-zero RPS is the bug we are hunting - dump the trace.
        KLog.d("ShooterRun_PowerZero", () -> "UNEXPECTED POWER 0.0 -> shooter: " + power +
                " | mode: " + shooterRunMode + " targetRPS: " + targetRPS + " currentRPS: " + currentRPS);

        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
        KLog.d("ShooterRun_PowerZero", "Power set to 0.0 - called from:");
        for (int i = 0; i < stackTrace.length; i++) {
            final int finalI = i;
            KLog.d("ShooterRun_PowerZero", () -> "  [" + finalI + "] " + stackTrace[finalI].toString());
        }
    }
}
