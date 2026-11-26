package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.OUT_OF_RANGE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig.hoodFlipDirection;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public static final String TAG = "Robot_Shooter";
    public static final String HOOD_TAG = "Robot_Hood";

    public static final double MM_TO_PIXEL_RATIO = 0.2608;

    public static final double GOAL_HEIGHT_MM = 750.0;//838
    public static final double GOAL_HEIGHT_PIXELS = GOAL_HEIGHT_MM * MM_TO_PIXEL_RATIO;

    public static final double HOOD_OFFSET = 0; // 0.25

    public final double TARGET_RPS_TOLERANCE = 1;

    private final OpModeUtilities opModeUtilities;

    private final KMotor shooterMotor;

    private final KServo hood;


    private final IShooterPredictor predictor;


    private double targetRPS;
    private double currentRPS;
    private double lastClampedRequest = Double.NaN;

    public Shooter(OpModeUtilities opModeUtilities) {

        this.opModeUtilities = opModeUtilities;
        DcMotor motor = opModeUtilities.getHardwareMap().dcMotor.get("shooter");
        motor.setDirection(ShooterConfig.SHOOTER_MOTOR_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor = new KMotor(motor, ShooterConfig.kp, ShooterConfig.ki, ShooterConfig.kd, ShooterConfig.kf);
        Servo hood = opModeUtilities.getHardwareMap().servo.get("hood");
        this.hood = new KServo(hood, KServo.AXON_MAX_SPEED_DEG_PER_SEC, 255, ShooterInterpolationConfig.MIN_HOOD, hoodFlipDirection)
                .setLimits(ShooterConfig.HOOD_DOWN_POS, ShooterConfig.HOOD_UP_POS);

        this.targetRPS = 0;

        predictor = new ShooterInterpolationDataLookup();
    }

    public double getHoodPosition() {
        return hood.getPosition();
    }

    public double getRPS() {
        return shooterMotor.getRPS();
    }

    public boolean isAtTargetRPS(double effectiveTolerance) {
        boolean isWithinTarget = shooterMotor.isAtTargetRPS(effectiveTolerance);
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> "event=atTargetCheck targetRps=" + targetRPS +
                    " currentRps=" + currentRPS +
                    " delta=" + (targetRPS - currentRPS) +
                    " power=" + shooterMotor.getPower() +
                    " voltage=" + SharedData.getVoltage() +
                    " withinTarget=" + isWithinTarget +
                    " tolerance=" + effectiveTolerance);
        }
        return isWithinTarget;
    }

    public boolean isRunning() {
        return Math.abs(shooterMotor.getPower()) > 0;
    }


    public KServo getHood() {
        return hood;
    }

    public KMotor getShooterMotor() {
        return shooterMotor;
    }

    /**
     * Calculate shooter parameters based on distance to target
     *
     * @param distanceMM distance to target in millimeters
     * @return shooter parameters containing RPS and hood position
     */
    public IShooterPredictor.ShooterParams getPrediction(double distanceMM) {
        if (distanceMM > OUT_OF_RANGE_THRESHOLD) {
            if (KLog.isDebug()) {
                KLog.d(TAG, () -> "event=outOfRangeFallback distanceMm=" + distanceMM +
                        " fallbackMm=" + ShooterInterpolationConfig.FALLBACK_OUT_OF_RANGE_DISTANCE_MM);
            }
            return predictor.predict(ShooterInterpolationConfig.FALLBACK_OUT_OF_RANGE_DISTANCE_MM);
        }
        return predictor.predict(distanceMM);
    }

    /**
     * Set hood position based on distance to target
     *
     * @param distanceMM distance to target in millimeters
     */
    private void setCalculatedHood(double distanceMM) {
        IShooterPredictor.ShooterParams params = getPrediction(distanceMM);

        // Note: This does NOT set motor power - use ShooterReady action for power ramping
        // This only sets the hood position
        // Hood offset is already included in predictor implementations
        if (KLog.isDebug()) {
            KLog.d(HOOD_TAG, () -> "event=setFromDistance distanceMm=" + distanceMM +
                    " from=" + hood.getPosition() + " to=" + params.hoodPosition);
        }
        hood.setPosition(params.hoodPosition);
    }

    public void stop() {
        shooterMotor.stopAndResetPID();
        KLog.d(TAG, "event=stop");
    }

    /**
     * Set motor power directly
     *
     * @param power motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    /**
     * Set target RPS using PID control. Every RPS command to the motor goes through here,
     * so the MAX_RPS clamp lives here.
     *
     * @param targetRPS desired rotations per second
     */
    public void goToRPS(double targetRPS) {
        this.targetRPS = clampRPS(targetRPS);
        currentRPS = shooterMotor.goToRPS(this.targetRPS);
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> "event=goToRps targetRps=" + this.targetRPS + " currentRps=" + currentRPS);
        }
    }

    /**
     * Clamp a requested RPS to [0, MAX_RPS]. Logs once per distinct out-of-range request (not every loop).
     */
    private double clampRPS(double requestedRPS) {
        double clamped = MathFunctions.clamp(requestedRPS, 0, ShooterInterpolationConfig.MAX_RPS);
        if (clamped != requestedRPS && requestedRPS != lastClampedRequest) {
            lastClampedRequest = requestedRPS;
            KLog.d(TAG, () -> "event=clampRps requested=" + requestedRPS + " max=" + ShooterInterpolationConfig.MAX_RPS);
        }
        return clamped;
    }

    /**
     * Continuously re-apply PID control toward the last commanded target RPS.
     *
     * Call this every loop iteration (even between actions) so the flywheel never coasts
     * or runs away during the gaps when no {@code ShooterRun} action is actively updating it.
     * When the target is 0 (e.g. after a STOP) the PID naturally floors to 0 power, so this
     * also keeps a stopped flywheel stopped.
     */
    public void holdTargetRPS() {
        goToRPS(targetRPS);
    }

    /**
     * Get the target RPS for a given position and target
     *
     * @param currentPosition current robot position from odometry (x, y, theta)
     * @param target          target point to shoot at
     * @return target RPS for the shooter
     */
    public double getDistance(Position currentPosition, Point target) {
        double dx = target.getX() - currentPosition.getX();
        double dy = target.getY() - currentPosition.getY();
        double distance = Math.sqrt((dx * dx) + (dy * dy));
        return distance;
    }

    public double getTargetRPS(Position currentPosition, Point target) {
        // Calculate distance between current position and target (ignoring theta)
        double distance = getDistance(currentPosition, target);

        IShooterPredictor.ShooterParams params = getPrediction(distance);

        if (KLog.isDebug()) {
            KLog.d(TAG, () -> "event=predict x=" + currentPosition.getX() + " y=" + currentPosition.getY() +
                    " targetX=" + target.getX() + " targetY=" + target.getY() +
                    " distanceMm=" + distance + " rps=" + params.shooterRPS + " hood=" + params.hoodPosition);
        }

        return params.shooterRPS;
    }

    /**
     * Update hood position based on current position and target
     *
     * @param currentPosition current robot position from odometry (x, y, theta)
     * @param target          target point to shoot at
     */
    public void updateHoodFromPosition(Position currentPosition, Point target) {
        // Calculate distance between current position and target (ignoring theta)
        double dx = target.getX() - currentPosition.getX();
        double dy = target.getY() - currentPosition.getY();
        double distance = Math.sqrt((dx * dx) + (dy * dy));

        // Use distance as xMM to set hood position
        setCalculatedHood(distance);
    }

    public static double mmToPixel(double mm) {
        return mm * MM_TO_PIXEL_RATIO;
    }

    public static double pixelToMM(double pixel) {
        return pixel / MM_TO_PIXEL_RATIO;
    }

    public void setTargetRPS(double targetRPS) {
        this.targetRPS = clampRPS(targetRPS);
        shooterMotor.setTargetRPS(this.targetRPS);

    }
}
