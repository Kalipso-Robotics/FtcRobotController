package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KCRServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

/**
 * One arm on one intake side, driven by an Axon Max in CR mode (configure "flower" as a Continuous Rotation Servo).
 * Driving into a Flower with the arm down knocks its balls into the intake.
 *
 * No position feedback: lower() runs FLOWER_DOWN_POWER for FLOWER_LOWER_MS, raise() runs FLOWER_UP_POWER for
 * FLOWER_RAISE_MS (IntakeConfig), then update() cuts the power, so update() must be called every loop
 * (IntakeSide.update() / Intake.update() do it). The arm must be up by hand when the OpMode is initialized.
 * ponytail: open-loop timed travel, and a reversal mid-move still runs its full time (pushes into the stop);
 * add the Axon analog feedback wire (AnalogInput) if timing proves unreliable. Tune with FlowerCRTuner.
 *
 * Interlock: the arm may be down only while its intake side is down, so lower() refuses unless
 * that side isDown(). The side's raise() raises the arm first (see IntakeSide).
 */
public class FlowerMech {
    public static final String TAG = "Robot_Flower";

    private final KCRServo servo;
    private final IntakeSide side;
    private final ElapsedTime sinceCommand = new ElapsedTime();

    private boolean isCommandedDown = false;
    private boolean isMoving = false; // a timed lower/raise is running
    private boolean isBlocked = false;

    public FlowerMech(OpModeUtilities opModeUtilities, IntakeSide side) {
        this(new KCRServo(opModeUtilities.getHardwareMap().crservo.get("flowerPivot"), IntakeConfig.FLOWER_FLIP_DIRECTION), side);
    }

    /** Direct-injection constructor, for a servo you have already pulled out of the hardware map (unit tests). */
    public FlowerMech(KCRServo servo, IntakeSide side) {
        this.servo = servo;
        this.side = side;
        servo.setPower(0); // no timed move here: nothing calls update() until the OpMode starts
        sinceCommand.reset();
        side.attachFlower(this);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=init flower=up");
    }

    /**
     * Lowers the arm, unless its intake side is not down: then it does nothing, logs a fault once,
     * and returns false. Returns true when the arm is commanded down (including already down).
     */
    public boolean lower() {
        if (isCommandedDown) {
            return true;
        }
        if (!side.isDown()) {
            if (!isBlocked) {
                isBlocked = true;
                KLog.e(TAG, "side=" + side.getSide() + " event=blocked reason=intakeUp");
            }
            return false;
        }
        isBlocked = false;
        isCommandedDown = true;
        startMove(IntakeConfig.FLOWER_DOWN_POWER);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=lower");
        return true;
    }

    public void raise() {
        isBlocked = false;
        if (!isCommandedDown) {
            return;
        }
        isCommandedDown = false;
        startMove(IntakeConfig.FLOWER_UP_POWER);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=raise");
    }

    private void startMove(double power) {
        isMoving = true;
        servo.setPower(power);
        sinceCommand.reset();
    }

    /** Call once per loop: cuts the power when the timed lower/raise is over. */
    public void update() {
        if (isMoving && isMoveTimeOver()) {
            isMoving = false;
            servo.setPower(0);
            KLog.d(TAG, () -> "side=" + side.getSide() + " event=moveDone flower=" + (isCommandedDown ? "down" : "up"));
        }
    }

    private boolean isMoveTimeOver() {
        return sinceCommand.milliseconds() >= (isCommandedDown ? IntakeConfig.FLOWER_LOWER_MS : IntakeConfig.FLOWER_RAISE_MS);
    }

    /** True while a timed lower/raise is running (until update() cuts the power). */
    public boolean isMoving() {
        return isMoving;
    }

    /**
     * Manual power (FlowerCRTuner), not timed: stays on until the next command. Keeps the interlock: power that is not toward up (same sign as FLOWER_UP_POWER,
     * or 0) is refused (returns false, logs event=blocked) unless this arm's intake side is down.
     * Power 0 counts as up only for the interlock, so the arm may be sagging; raise() before driving.
     */
    public boolean setPower(double power) {
        boolean towardUp = power == 0 || Math.signum(power) == Math.signum(IntakeConfig.FLOWER_UP_POWER);
        if (!towardUp && !side.isDown()) {
            KLog.e(TAG, "side=" + side.getSide() + " event=blocked reason=intakeUp power=" + power);
            return false;
        }
        isBlocked = false;
        if (isCommandedDown == towardUp) {
            sinceCommand.reset();
        }
        isCommandedDown = !towardUp;
        isMoving = false;
        servo.setPower(power);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=setPower power=" + power);
        return true;
    }

    /** True once commanded down and FLOWER_LOWER_MS has passed. */
    public boolean isDown() {
        return isCommandedDown && isMoveTimeOver();
    }

    /** True once commanded up and FLOWER_RAISE_MS has passed. */
    public boolean isUp() {
        return !isCommandedDown && isMoveTimeOver();
    }

    /** Last commanded power (after flip), for telemetry. */
    public double getPower() {
        return servo.getPower();
    }
}
