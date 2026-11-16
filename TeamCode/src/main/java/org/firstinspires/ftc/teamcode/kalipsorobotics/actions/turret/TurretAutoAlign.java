package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.DoneStateAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
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

    private final Point targetPoint;
    private double toleranceDeg = 1.5;
    private boolean isWithinRange = false;
    private double deltaAngleDeg;

    private final AllianceColor allianceColor;

    public TurretAutoAlign(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.dependentActions.add(new DoneStateAction());
        this.allianceColor = allianceColor;

        targetPoint = new Point(TurretConfig.X_INIT_SETUP_MM, TurretConfig.Y_INIT_SETUP_MM * allianceColor.getPolarity());

        this.targetAngleRad = 0;
    }

    /** Servo finished its estimated travel last loop, and the new target is within tolerance of where it is (so not clamped beyond tolerance). */
    public boolean isWithinRange() {
        return isWithinRange;
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
            KLog.d(TAG, () -> "event=start trimDeg=" + TurretConfig.TRIM_DEG);
        }

        targetAngleRad = calculateTargetAngleRad(targetPoint, SharedData.getOdometryWheelIMUPosition(), TurretConfig.TRIM_DEG);
        SOTM.Solution sotm = SOTM.usableSolution();
        if (sotm != null) {
            targetAngleRad = calculateTargetAngleRad(sotm.aimHeadingRad, sotm.releaseHeadingRad, TurretConfig.TRIM_DEG);
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


    public static double calculateTargetAngleRad(Point targetPoint, Position currentPosition, double trimDeg) {
        double angleTargetRadian = Math.atan2(targetPoint.getY() - currentPosition.getY(), targetPoint.getX() - currentPosition.getX());
        return calculateTargetAngleRad(angleTargetRadian, currentPosition.getTheta(), trimDeg);
    }

    /** Field aim angle and robot heading to turret angle: (aim - heading + trim), wrapped. */
    public static double calculateTargetAngleRad(double aimRad, double headingRad, double trimDeg) {
        return MathFunctions.angleWrapRad(aimRad - headingRad + Math.toRadians(trimDeg));
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
