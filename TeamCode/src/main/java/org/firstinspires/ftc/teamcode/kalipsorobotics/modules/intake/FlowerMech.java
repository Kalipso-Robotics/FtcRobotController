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
 * No position feedback: lower()/raise() set IntakeConfig.FLOWER_DOWN_POWER / FLOWER_UP_POWER and keep it applied,
 * pushing the arm into its hard stop. isDown()/isUp() turn true FLOWER_MOVE_MS after the command.
 * ponytail: open-loop timed travel; add the Axon analog feedback wire (AnalogInput) if timing proves unreliable.
 * Tune with FlowerCRTuner.
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
    private boolean isBlocked = false;

    public FlowerMech(OpModeUtilities opModeUtilities, IntakeSide side) {
        this(new KCRServo(opModeUtilities.getHardwareMap().crservo.get("flowerPivot"), IntakeConfig.FLOWER_FLIP_DIRECTION), side);
    }

    /** Direct-injection constructor, for a servo you have already pulled out of the hardware map (unit tests). */
    public FlowerMech(KCRServo servo, IntakeSide side) {
        this.servo = servo;
        this.side = side;
        servo.setPower(IntakeConfig.FLOWER_UP_POWER);
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
        servo.setPower(IntakeConfig.FLOWER_DOWN_POWER);
        sinceCommand.reset();
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=lower");
        return true;
    }

    public void raise() {
        isBlocked = false;
        if (!isCommandedDown) {
            return;
        }
        isCommandedDown = false;
        servo.setPower(IntakeConfig.FLOWER_UP_POWER);
        sinceCommand.reset();
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=raise");
    }

    /**
     * Manual power (FlowerCRTuner). Keeps the interlock: power that is not toward up (same sign as FLOWER_UP_POWER,
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
        servo.setPower(power);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=setPower power=" + power);
        return true;
    }

    /** True once commanded down and FLOWER_MOVE_MS has passed. */
    public boolean isDown() {
        return isCommandedDown && sinceCommand.milliseconds() >= IntakeConfig.FLOWER_MOVE_MS;
    }

    /** True once commanded up and FLOWER_MOVE_MS has passed. */
    public boolean isUp() {
        return !isCommandedDown && sinceCommand.milliseconds() >= IntakeConfig.FLOWER_MOVE_MS;
    }

    /** Last commanded power (after flip), for telemetry. */
    public double getPower() {
        return servo.getPower();
    }
}
