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


public class TurretAutoAlign extends Action {
    public static final String TAG = "Shooting_TurretAlign";

    OpModeUtilities opModeUtilities;

    Turret turret;

    private double targetAngleRad;

    private Point targetPoint;
    private double toleranceDeg = 1.5;
    private boolean isWithinRange = false;
    private double deltaAngleDeg;

    public TurretAutoAlign(OpModeUtilities opModeUtilities, Turret turret, Point target) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.dependentActions.add(new DoneStateAction());
        this.targetPoint = target;

        this.targetAngleRad = 0;
    }

    /** Servo finished its estimated travel last loop, and the new target is within tolerance of where it is (so not clamped beyond tolerance). */
    public boolean isWithinRange() {
        return isWithinRange;
    }

    /** Aim at a different field point from the next update on. */
    public void setTargetPoint(Point target) {
        this.targetPoint = target;
    }

    public void stop() {
        turret.stop();
        KLog.d(TAG, "event=stop");
    }

    public Turret getTurret() {
        return turret;
    }

    @Override
    protected void update() {
        if (!opModeUtilities.getOpMode().opModeIsActive() && !opModeUtilities.getOpMode().opModeInInit()) {
            KLog.d(TAG, "event=skip reason=opModeNotActive");
            turret.stop();
            return;
        }
        if (!hasStarted) {
            hasStarted = true;
            KLog.d(TAG, () -> "event=start offsetDeg=" + TurretConfig.TURRET_OFFSET_DEG);
        }

        targetAngleRad = calculateTargetAngleRad(targetPoint, SharedData.getOdometryWheelIMUPosition(), TurretConfig.TURRET_OFFSET_DEG);
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
