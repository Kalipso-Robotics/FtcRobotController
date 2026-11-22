package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * One side of the double-sided intake (modeled on Stopper). One motor drives this side's intake
 * wheels and its two transfer wheel sets; one servo pivots the side down to the ground or up.
 * There are two instances, A (front, +X) and B (back). The hardware is identical, so the only
 * per-side difference here is which device names and config positions are read.
 *
 * Flower interlock: if the Flower is on this side, raise() raises the flower first and leaves the
 * pivot down until the flower is up. The pending raise then finishes on a later raise() or update().
 */
public class IntakeSide {
    public static final String TAG = "Robot_Intake";

    private final BeamBreakPosition.IntakeSide side;
    private final DcMotor motor;
    private final KServo pivot;
    private final double downPos;
    private final double upPos;

    private FlowerMech flowerMech = null;
    private boolean isCommandedDown = false;
    private boolean isRaisePending = false;
    private double power = 0;

    public IntakeSide(OpModeUtilities opModeUtilities, BeamBreakPosition.IntakeSide side) {
        this(side,
                opModeUtilities.getHardwareMap().dcMotor.get("intake" + side.name()),
                new KServo(opModeUtilities.getHardwareMap().servo.get("intakePivot" + side.name()),
                        KServo.AXON_MAX_SPEED_DEG_PER_SEC, 255, 0, false)); // TODO-measure: servo model, speed and range
    }

    /** Direct-injection constructor, for hardware you have already pulled out of the hardware map (unit tests). */
    public IntakeSide(BeamBreakPosition.IntakeSide side, DcMotor motor, KServo pivot) {
        this.side = side;
        this.motor = motor;
        this.pivot = pivot;
        boolean isA = side == BeamBreakPosition.IntakeSide.A;
        this.downPos = isA ? ModuleConfig.INTAKE_PIVOT_A_DOWN_POS : ModuleConfig.INTAKE_PIVOT_B_DOWN_POS;
        this.upPos = isA ? ModuleConfig.INTAKE_PIVOT_A_UP_POS : ModuleConfig.INTAKE_PIVOT_B_UP_POS;
        motor.setDirection(isA ? ModuleConfig.INTAKE_MOTOR_A_DIRECTION : ModuleConfig.INTAKE_MOTOR_B_DIRECTION);
        // Start raised: the side is spring loaded to the ground, so lowered would stick out past the starting size.
        pivot.setTargetPosition(upPos);
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
        pivot.setTargetPosition(downPos);
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
        pivot.setTargetPosition(upPos);
        KLog.d(TAG, () -> "side=" + side + " event=raise");
    }

    /** Call once per loop: finishes a raise that was waiting for the flower. */
    public void update() {
        if (isRaisePending) {
            raise();
        }
    }

    public void run(double power) {
        motor.setPower(power);
        if (power != this.power) {
            this.power = power;
            KLog.d(TAG, () -> "side=" + side + " event=run power=" + power);
        }
    }

    public void stop() {
        run(0);
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
        boolean toDown = Math.abs(position - downPos) < 1e-6; // stepped values miss DOWN by ~1e-16
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

    public BeamBreakPosition.IntakeSide getSide() {
        return side;
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

    /** Motor speed in ticks/s (0 if the motor cannot report it). */
    public double getVelocity() {
        return motor instanceof DcMotorEx ? ((DcMotorEx) motor).getVelocity() : 0;
    }
}
