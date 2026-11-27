package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PIDFController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KMotor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * One side of the double-sided intake (modeled on Stopper). One motor drives this side's intake
 * wheels and its two transfer wheel sets; one servo pivots the side down to the ground or up.
 * There are two instances, A (front, +X) and B (back). The hardware is identical; everything that differs per
 * side (device names, motor direction, pivot positions, feed gains) is in its IntakeModuleConfig.
 *
 * Flower interlock: if the Flower is on this side, raise() raises the flower first and leaves the
 * pivot down until the flower is up. The pending raise then finishes on a later raise() or update().
 */
public class IntakeModule {
    public static final String TAG = "Robot_Intake";

    private final IntakeSide side;
    private final DcMotor motor;
    private final KServo pivot;
    private final IntakeModuleConfig intakeModuleConfig;

    private FlowerMech flowerMech = null;
    private boolean isCommandedDown = false;
    private boolean isRaisePending = false;
    private double power = 0;
    private final PIDFController feedController;

    public IntakeModule(OpModeUtilities opModeUtilities, IntakeSide side, IntakeModuleConfig intakeModuleConfig) {
        this(side,
                opModeUtilities.getHardwareMap().dcMotor.get(intakeModuleConfig.motorName),
                new KServo(opModeUtilities.getHardwareMap().servo.get(intakeModuleConfig.pivotName),
                        KServo.AXON_MAX_SPEED_DEG_PER_SEC, 255, 0, false), // TODO-measure: servo model, speed and range
                intakeModuleConfig);
    }

    /** Direct-injection constructor, for hardware you have already pulled out of the hardware map (unit tests). */
    public IntakeModule(IntakeSide side, DcMotor motor, KServo pivot, IntakeModuleConfig intakeModuleConfig) {
        this.side = side;
        this.motor = motor;
        this.pivot = pivot;
        this.intakeModuleConfig = intakeModuleConfig;
        this.feedController = PIDFController.flywheel("Intake_" + side, 0, 0, 0, 0).build(); // gains read per call in runRPS
        motor.setDirection(intakeModuleConfig.direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Start raised: the side is spring loaded to the ground, so lowered would stick out past the starting size.
        pivot.setPosition(intakeModuleConfig.pivotDownPos); //dont remove, weirf bug, needs to be here
        pivot.setPosition(intakeModuleConfig.initPivotPos);
        KLog.d(TAG, () -> "pivotPosition: " + pivot.getPosition());
        KLog.d(TAG, () -> "side=" + side + " event=init pivot=up");
    }

    /** Called by the Flower constructor, so a flower is always attached to the side it checks. */
    void attachFlower(FlowerMech flowerMech) {
        if (this.flowerMech != null) {
            throw new IllegalStateException("side " + side + " already has a flower");
        }
        this.flowerMech = flowerMech;
    }

    public void lower() {
        if (isRaisePending) {
            isRaisePending = false;
            KLog.d(TAG, () -> "side=" + side + " event=raiseCancelled reason=lower");
        }
        if (isCommandedDown) {
            return;
        }
        isCommandedDown = true;
        pivot.setTargetPosition(intakeModuleConfig.pivotDownPos);
        KLog.d(TAG, () -> "side=" + side + " event=lower");
    }

    /** Raises the side. If this side's flower is not up, raises the flower first and finishes later. */
    public void raise() {
        if (!isCommandedDown) {
            return;
        }
        if (flowerMech != null && !flowerMech.isUp()) {
            if (!isRaisePending) {
                isRaisePending = true;
                KLog.e(TAG, "side=" + side + " event=raiseDeferred reason=flowerNotUp");
            }
            flowerMech.raise();
            return;
        }
        isRaisePending = false;
        isCommandedDown = false;
        pivot.setTargetPosition(intakeModuleConfig.pivotUpPos);
        KLog.d(TAG, () -> "side=" + side + " event=raise");
    }

    /** Call once per loop: ends the flower's timed move and finishes a raise that was waiting for the flower. */
    public void update() {
        if (flowerMech != null) {
            flowerMech.update();
        }
        if (isRaisePending) {
            raise();
        }
    }

    /** Open loop. Also ends any closed loop (runRPS), so the next one starts clean. */
    public void run(double power) {
        feedController.reset();
        motor.setPower(power);
        if (power != this.power) {
            this.power = power;
            KLog.d(TAG, () -> "side=" + side + " event=run power=" + power);
        }
    }

    public void stop() {
        run(0);
    }

    /**
     * Closed loop on this side's encoder, forward only, in motor RPS: power = kF * target + kP * error, times
     * KMotor.voltageScale (same battery compensation as the shooter). Gains are this side's config kF / kP
     * (power per RPS), read every call so Dashboard edits apply live. Call every loop; run() ends it.
     */
    public void runRPS(double targetRPS) {
        runRPS(targetRPS, intakeModuleConfig.kF, intakeModuleConfig.kP);
    }

    /** runRPS with these gains instead of the config's (gain tuning: MotorPowerRPSDataCollector). Returns the RPS it read. */
    public double runRPS(double targetRPS, double kf, double kp) {
        feedController.setKf(kf);
        feedController.setKp(kp);
        double rps = getRPS();
        double newPower = Math.min(1, feedController.calculate(rps, targetRPS) * KMotor.voltageScale(SharedData.getVoltage()));
        motor.setPower(newPower);
        power = newPower;
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format("side=%s event=runRPS targetRps=%.2f rps=%.2f power=%.3f",
                    side, targetRPS, rps, newPower));
        }
        return rps;
    }

    /** kF (power per RPS) that would give today's open-loop power at today's speed: power / (rps * voltageScale); NaN when not moving forward. */
    public double estimateKf() {
        double rps = getRPS();
        return rps > 0.1 ? power / (rps * KMotor.voltageScale(SharedData.getVoltage())) : Double.NaN;
    }

    /** True once the pivot was commanded down, is not waiting to raise, and has had time to get there. */
    public boolean isDown() {
        return isCommandedDown && !isRaisePending && pivot.isDone();
    }

    /** True once commanded up, not waiting to raise, and the pivot has had time to get there. */
    public boolean isUp() {
        return !isCommandedDown && !isRaisePending && pivot.isDone();
    }

    /**
     * Manual pivot control (BasicTeleop). Keeps the Flower interlock: refuses to leave DOWN while this side's
     * Flower is not up (logs event=blocked, returns false). Any position other than DOWN counts as "not down".
     */
    public boolean setPivotPosition(double position) {
        boolean toDown = Math.abs(position - intakeModuleConfig.pivotDownPos) < 1e-6; // stepped values miss DOWN by ~1e-16
        if (isCommandedDown && !toDown && flowerMech != null && !flowerMech.isUp()) {
            KLog.e(TAG, "side=" + side + " event=blocked reason=flowerNotUp pos=" + position);
            return false;
        }
        isRaisePending = false;
        isCommandedDown = toDown;
        pivot.setTargetPosition(position);
        KLog.d(TAG, () -> "side=" + side + " event=setPivot pos=" + position);
        return true;
    }

    public IntakeSide getSide() {
        return side;
    }

    public IntakeModuleConfig getConfig() {
        return intakeModuleConfig;
    }

    /** Last commanded pivot servo position, for telemetry. */
    public double getPivotPosition() {
        return pivot.getTargetPosition();
    }

    public double getPower() {
        return power;
    }

    /** Motor current in mA (0 if the motor cannot report it). Not in the bulk cache: one extra hub read per call. */
    public double getCurrentMA() {
        return motor instanceof DcMotorEx ? ((DcMotorEx) motor).getCurrent(CurrentUnit.MILLIAMPS) : 0;
    }

    /** True when this side's RPS is within toleranceRPS of targetRPS. */
    public boolean isAtRPS(double targetRPS, double toleranceRPS) {
        return Math.abs(getRPS() - targetRPS) <= toleranceRPS;
    }

    /** Motor output speed in rev/s (ModuleConfig.INTAKE_TICKS_PER_REV); + = intake-in / feed direction. */
    public double getRPS() {
        return getVelocity() / ModuleConfig.INTAKE_TICKS_PER_REV;
    }

    /** Raw encoder count (direction applied, like getVelocity). */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /** Motor speed in ticks/s (0 if the motor cannot report it). */
    public double getVelocity() {
        return motor instanceof DcMotorEx ? ((DcMotorEx) motor).getVelocity() : 0;
    }
}
