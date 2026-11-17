package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;


public class TurretAutoAlignTeleOp extends Action {
    public static final String TAG = TurretAutoAlign.TAG;

    OpModeUtilities opModeUtilities;
    Turret turret;
    private TurretRunMode turretRunMode;
    private double targetAngleRad;
    private double toleranceDeg = 2;
    private boolean isWithinRange = false;

    Point targetPoint;
    private double deltaAngleDeg;


    public TurretAutoAlignTeleOp(OpModeUtilities opModeUtilities, Turret turret) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.dependentActions.add(new DoneStateAction());
        this.turretRunMode = TurretRunMode.STOP; // initial mode
        this.targetAngleRad = 0;
        targetPoint = FieldConfig.bTargetPoint; // TeleOp replaces this every loop with the Cell on the robot's end of the Hive
    }

    /** The field point the turret aims at (our alliance's own frame). */
    public void setTargetPoint(Point targetPoint) {
        this.targetPoint = targetPoint;
    }

    /** STOP: always true. RUN_USING_ODOMETRY: servo finished its estimated travel last loop, and the new target is within tolerance of where it is. */
    public boolean isWithinRange() {
        return isWithinRange;
    }

    public Turret getTurret() {
        return turret;
    }

    @Override
    protected void update() {
        if (!opModeUtilities.getOpMode().opModeIsActive() && !opModeUtilities.getOpMode().opModeInInit()) {
            turret.stop();
            return;
        }

        if (!hasStarted) {
            hasStarted = true;
            KLog.d(TAG, () -> "event=start mode=" + turretRunMode);
        }

        switch (turretRunMode) {
            case STOP:
                turret.stop();
                isWithinRange = true;
                break;
            case RUN_USING_ODOMETRY:
                updateAlignToTarget();
                break;
        }
    }

    private void updateAlignToTarget() {
        Position currentPos = SharedData.peekOdometryWheelIMUPosition();

        targetAngleRad = TurretAutoAlign.calculateTargetAngleRad(targetPoint, currentPos, TurretConfig.TRIM_DEG);
        SOTM.Solution sotm = SOTM.usableSolution();
        if (sotm != null) {
            targetAngleRad = TurretAutoAlign.calculateTargetAngleRad(sotm.aimHeadingRad, sotm.releaseHeadingRad, TurretConfig.TRIM_DEG);
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
        return this.turretRunMode;
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
