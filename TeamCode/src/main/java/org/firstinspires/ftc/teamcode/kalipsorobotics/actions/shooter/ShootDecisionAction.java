package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;

/**
 * Decides where to shoot from and what to aim at: we always shoot into the Cell of the Hive that is UP.
 *   Cell A up              -> shoot from SHOOT_TIP_ZONE_2, aim at TARGET_POINT_1
 *   Cell B up (or unknown) -> shoot from SHOOT_TIP_ZONE_1, aim at TARGET_POINT_2
 */
public class ShootDecisionAction extends Action {

    // Where we shoot from on each side (same spots as RedThreeTipSoloAuto). Position angles are in radians.
    public static final Position SHOOT_TIP_ZONE_1 = new Position(1037, 397, Math.toRadians(289.1));
    public static final Position SHOOT_TIP_ZONE_2 = new Position(300, 2949, Math.toRadians(102.6));

    // The answers
    private Position shootPosition = SHOOT_TIP_ZONE_1;
    private Point targetPoint = ShooterConfig.TARGET_POINT_2;
    private HiveTagLocalizer.Cell aimedCell = HiveTagLocalizer.Cell.B;

    public ShootDecisionAction() {
        this.name = "ShootDecisionAction";
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        decide();
        isDone = true;
    }

    /** Looks at which Cell is up and picks the shoot point and target. Call this once per trip. */
    public void decide() {
        boolean cellAIsUp = SharedData.getHiveUpCell() == HiveTagLocalizer.Cell.A;

        if (cellAIsUp) {
            shootPosition = SHOOT_TIP_ZONE_2;
            targetPoint = ShooterConfig.TARGET_POINT_1;
            aimedCell = HiveTagLocalizer.Cell.A;
        } else {
            shootPosition = SHOOT_TIP_ZONE_1;
            targetPoint = ShooterConfig.TARGET_POINT_2;
            aimedCell = HiveTagLocalizer.Cell.B;
        }
    }

    public Position getShootPosition() {
        return shootPosition;
    }

    public Point getTargetPoint() {
        return targetPoint;
    }

    /** The Cell we aimed at last time decide() ran. */
    public HiveTagLocalizer.Cell getAimedCell() {
        return aimedCell;
    }
}
