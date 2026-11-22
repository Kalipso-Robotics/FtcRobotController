package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * One servo turns the turret through gears (see TurretConfig). There is no encoder, so the
 * angle we report is the last one we commanded. Servo wrapped in KServo the same way Stopper does.
 *
 * Angle convention (same as last year's motor turret): 0 = straight ahead, positive turns
 * the same way as a positive robot heading theta (aim angle - robot heading).
 */
public class Turret {
    public static final String TAG = "Robot_Turret";

    private static Turret single_instance = null;

    private OpModeUtilities opModeUtilities;

    private KServo turretServo;

    private double commandedDeg = 0;
    private boolean isClamped = false;
    // KServo restarts its estimate on every re-target, so a tiny re-target would hide a big swing still in progress.
    private double busyUntilMs = 0;

    private Turret(OpModeUtilities opModeUtilities) {
        resetHardwareMap(opModeUtilities, this);
        setAngleDeg(0);
    }

    /** Direct-injection constructor, for a servo you have already pulled out of the hardware map (unit tests). */
    Turret(KServo turretServo) {
        this.turretServo = turretServo;
        setAngleDeg(0);
    }

    public static synchronized Turret getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new Turret(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities, single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(OpModeUtilities opModeUtilities, Turret turret) {
        turret.opModeUtilities = opModeUtilities;
        Servo servo = opModeUtilities.getHardwareMap().servo.get("turret");
        turret.turretServo = new KServo(servo, KServo.AXON_MAX_SPEED_DEG_PER_SEC, // TODO-measure: servo model not chosen yet
                TurretConfig.SERVO_RANGE_DEG, TurretConfig.SERVO_ZERO_POS, false);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public void setAngleRad(double turretRad) {
        setAngleDeg(Math.toDegrees(turretRad));
    }

    /** Points the turret, clamped to the reachable range. Logs a fault once each time a target goes out of range. */
    public void setAngleDeg(double turretDeg) {
        double clampedDeg = clampTurretDeg(turretDeg);
        boolean wasClamped = isClamped;
        isClamped = clampedDeg != turretDeg;
        if (isClamped && !wasClamped) {
            KLog.e(TAG, String.format("event=clamp requestedDeg=%.1f limitDeg=%.1f", turretDeg, clampedDeg));
        } else if (!isClamped && wasClamped) {
            KLog.d(TAG, () -> String.format("event=inRange targetDeg=%.1f", clampedDeg));
        }
        commandedDeg = clampedDeg;
        double servoPos = servoPosFromTurretDeg(clampedDeg);
        busyUntilMs = Math.max(busyUntilMs, System.currentTimeMillis() + turretServo.estimateFinishTimeMs(turretServo.getPosition(), servoPos));
        turretServo.setTargetPosition(servoPos);
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format("targetDeg=%.2f servoPos=%.4f", clampedDeg, servoPos));
        }
    }

    /** Last commanded angle after clamping (no encoder). */
    public double getAngleDeg() {
        return commandedDeg;
    }

    public double getAngleRad() {
        return Math.toRadians(commandedDeg);
    }

    /** True if the last target was outside the reachable range and got clamped. */
    public boolean isClamped() {
        return isClamped;
    }

    /** True while the servo is still within its estimated travel time, covering every target since it last settled. */
    public boolean isMoving() {
        return System.currentTimeMillis() < busyUntilMs;
    }

    /** A servo holds its last position; nothing to do. */
    public void stop() {
    }

    public static double clampTurretDeg(double turretDeg) {
        return MathFunctions.clamp(turretDeg, -TurretConfig.MAX_TURRET_DEG, TurretConfig.MAX_TURRET_DEG);
    }

    public static double servoPosFromTurretDeg(double turretDeg) {
        double servoDeg = turretDeg / TurretConfig.TURRET_DEG_PER_SERVO_DEG;
        return KServo.clampServoPos(TurretConfig.SERVO_ZERO_POS + TurretConfig.SERVO_DIRECTION * servoDeg / TurretConfig.SERVO_RANGE_DEG);
    }

    public static double turretDegFromServoPos(double servoPos) {
        double servoDeg = (servoPos - TurretConfig.SERVO_ZERO_POS) * TurretConfig.SERVO_RANGE_DEG / TurretConfig.SERVO_DIRECTION;
        return servoDeg * TurretConfig.TURRET_DEG_PER_SERVO_DEG;
    }
}
