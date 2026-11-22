package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * Moves the hood to a fixed raw servo position, clamped to MIN_HOOD..MAX_HOOD, done when the servo has had time to get there.
 * NOTE: a running ShooterRun sets the hood every loop and overwrites this.
 * Auto: {@code new HoodSetAction(shooter, ShooterInterpolationConfig.MIN_HOOD)}
 * Pattern source: CloseStopperAction (one KServoAutoAction).
 */
public class HoodSetAction extends KActionSet {
    public static final String TAG = "Shooting_Hood";

    public HoodSetAction(Shooter shooter, double hoodPos) {
        double clamped = MathFunctions.clamp(hoodPos, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
        KLog.d(TAG, () -> "event=start requested=" + hoodPos + " pos=" + clamped);
        KServoAutoAction setHood = new KServoAutoAction(shooter.getHood(), clamped);
        setHood.setName("hoodSet");
        this.addAction(setHood);
    }
}
