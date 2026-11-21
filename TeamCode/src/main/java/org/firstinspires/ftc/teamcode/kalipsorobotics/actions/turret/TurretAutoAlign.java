package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Points the turret at a field point on every update. Used by auto and TeleOp.
 * Target: a fixed point (constructor or setTargetPoint), or null = SharedData.getTargetPoint() (the Hive Cell on
 * the robot's end of the Hive). RUN_USING_ODOMETRY aims; STOP leaves the turret where it is and counts as aligned.
 */
public class TurretAutoAlign extends Action {
    public static final String TAG = "Shooting_TurretAlign";

    private final OpModeUtilities opModeUtilities;
    private final Turret turret;
    private TurretRunMode turretRunMode = TurretRunMode.RUN_USING_ODOMETRY;
    private Point targetPoint; // null = follow SharedData.getTargetPoint()
    private double targetAngleRad = 0;
    private double toleranceDeg = 1.5; // TODO-measure
    private boolean isWithinRange = false;
    private double deltaAngleDeg;

    /** Aims at SharedData.getTargetPoint(), the Hive Cell on the robot's end of the Hive. */
    public TurretAutoAlign(OpModeUtilities opModeUtilities, Turret turret) {
        this(opModeUtilities, turret, null);
    }

    /** Aims at a fixed field point (our alliance's own frame); null = SharedData.getTargetPoint(). */
    public TurretAutoAlign(OpModeUtilities opModeUtilities, Turret turret, Point target) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.dependentActions.add(new DoneStateAction());
        this.targetPoint = target;
    }

    /** Aim at a different field point from the next update on; null = SharedData.getTargetPoint(). */
    public void setTargetPoint(Point target) {
        this.targetPoint = target;
    }

    /** The point aimed at right now. */
    public Point getTargetPoint() {
        return targetPoint != null ? targetPoint : SharedData.getTargetPoint();
    }

    /** STOP: always true. RUN_USING_ODOMETRY: servo finished its estimated travel last loop, and the new target is within tolerance of where it is. */
    public boolean isWithinRange() {
        return isWithinRange;
    }

    public Turret getTurret() {
        return turret;
    }

    /** Stop aiming; the turret stays where it is until setTurretRunMode(RUN_USING_ODOMETRY). */
    public void stop() {
        setTurretRunMode(TurretRunMode.STOP);
    }

    public void setTurretRunMode(TurretRunMode turretRunMode) {
        TurretRunMode prevMode = this.turretRunMode;
        this.turretRunMode = turretRunMode;
        if (prevMode != turretRunMode) {
            KLog.d(TAG, () -> "event=mode from=" + prevMode + " to=" + turretRunMode);
            update();
        }
    }

    public TurretRunMode getTurretRunMode() {
        return turretRunMode;
    }

    @Override
    protected void update() {
        if (!opModeUtilities.getOpMode().opModeIsActive() && !opModeUtilities.getOpMode().opModeInInit()) {
            turret.stop();
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            KLog.d(TAG, () -> "event=start mode=" + turretRunMode + " offsetDeg=" + TurretConfig.TURRET_OFFSET_DEG);
        }

        switch (turretRunMode) {
            case STOP:
                turret.stop();
                isWithinRange = true;
                break;
            case RUN_USING_ODOMETRY:
                alignToTarget();
                break;
        }
    }

    private void alignToTarget() {
        targetAngleRad = calculateTargetAngleRad(getTargetPoint(), SharedData.peekOdometryWheelIMUPosition(), TurretConfig.TURRET_OFFSET_DEG);
        SOTM.Solution sotm = SOTM.usableSolution();
        if (sotm != null) {
            targetAngleRad = calculateTargetAngleRad(sotm.aimHeadingRad, sotm.releaseHeadingRad, TurretConfig.TURRET_OFFSET_DEG);
        }

        // No encoder: the turret is where we last commanded it once its estimated travel is over.
        boolean wasMoving = turret.isMoving();
        deltaAngleDeg = Math.toDegrees(targetAngleRad) - turret.getAngleDeg();
        turret.setAngleRad(targetAngleRad);

        boolean wasWithinRange = isWithinRange;
        isWithinRange = !wasMoving && Math.abs(deltaAngleDeg) < Math.abs(toleranceDeg);
        if (isWithinRange != wasWithinRange) {
            KLog.d(TAG, () -> String.format("event=%s targetDeg=%.1f deltaDeg=%.1f",
                    isWithinRange ? "aligned" : "notAligned", Math.toDegrees(targetAngleRad), deltaAngleDeg));
        }
    }

    public static double calculateTargetAngleRad(Point targetPoint, Position currentPosition, double offsetDeg) {
        double angleTargetRadian = Math.atan2(targetPoint.getY() - currentPosition.getY(), targetPoint.getX() - currentPosition.getX());
        return calculateTargetAngleRad(angleTargetRadian, currentPosition.getTheta(), offsetDeg);
    }

    /** Field aim angle and robot heading to turret angle: (aim - heading + trim), wrapped. */
    public static double calculateTargetAngleRad(double aimRad, double headingRad, double offsetDeg) {
        return MathFunctions.angleWrapRad(aimRad - headingRad + Math.toRadians(offsetDeg));
    }

    public double getTargetAngleRad() {
        return targetAngleRad;
    }

    public void setToleranceDeg(double newToleranceDeg) {
        toleranceDeg = newToleranceDeg;
    }

    /** Target angle minus where the turret was last commanded (includes any clamp shortfall). */
    public double getDeltaAngleDeg() {
        return deltaAngleDeg;
    }
}
