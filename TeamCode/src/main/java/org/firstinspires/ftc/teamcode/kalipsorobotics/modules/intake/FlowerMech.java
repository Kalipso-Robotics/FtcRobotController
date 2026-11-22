package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

/**
 * One servo arm on one intake side (modeled on Stopper). Driving into a Flower with the arm down
 * knocks its balls into the intake.
 *
 * Interlock: the arm may be down only while its intake side is down, so lower() refuses unless
 * that side isDown(). The side's raise() raises the arm first (see IntakeSide).
 */
public class FlowerMech {
    public static final String TAG = "Robot_Flower";

    private static final double POSITION_TOLERANCE = 1e-6;

    private final KServo servo;
    private final IntakeSide side;

    private boolean isCommandedDown = false;
    private boolean isBlocked = false;

    public FlowerMech(OpModeUtilities opModeUtilities, IntakeSide side) {
        this(new KServo(opModeUtilities.getHardwareMap().servo.get("flower"),
                KServo.AXON_MAX_SPEED_DEG_PER_SEC, 255, 0, false), side); // TODO-measure: servo model, speed and range
    }

    /** Direct-injection constructor, for a servo you have already pulled out of the hardware map (unit tests). */
    public FlowerMech(KServo servo, IntakeSide side) {
        this.servo = servo;
        this.side = side;
        servo.setTargetPosition(ModuleConfig.FLOWER_UP_POS);
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
        servo.setTargetPosition(ModuleConfig.FLOWER_DOWN_POS);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=lower");
        return true;
    }

    public void raise() {
        isBlocked = false;
        if (!isCommandedDown) {
            return;
        }
        isCommandedDown = false;
        servo.setTargetPosition(ModuleConfig.FLOWER_UP_POS);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=raise");
    }

    /**
     * Manual arm control (BasicTeleop). Keeps the interlock: any position other than UP is refused (returns false,
     * logs event=blocked) unless this arm's intake side is down.
     */
    public boolean setPosition(double position) {
        boolean toUp = Math.abs(position - ModuleConfig.FLOWER_UP_POS) < POSITION_TOLERANCE; // stepped values miss UP by ~1e-16
        if (!toUp && !side.isDown()) {
            KLog.e(TAG, "side=" + side.getSide() + " event=blocked reason=intakeUp pos=" + position);
            return false;
        }
        isBlocked = false;
        isCommandedDown = !toUp;
        servo.setTargetPosition(position);
        KLog.d(TAG, () -> "side=" + side.getSide() + " event=setPosition pos=" + position);
        return true;
    }

    /** True once commanded down and the servo has had time to get there. */
    public boolean isDown() {
        return isCommandedDown && servo.isDone();
    }

    /** True once commanded up and the servo has had time to get there. */
    public boolean isUp() {
        return !isCommandedDown && servo.isDone();
    }

    /** Last commanded servo position, for telemetry. */
    public double getPosition() {
        return servo.getTargetPosition();
    }


}
