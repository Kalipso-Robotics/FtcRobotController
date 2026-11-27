package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PidNav;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Drives straight forward (positive mm) or backward (negative mm) from wherever the robot is
 * when the action starts, holding the starting heading, using only PID on odometry.
 *
 * Pattern source: PurePursuitAction.targetPosition() (same robot-frame error math, PidNav
 * controllers and mecanum power mixing), minus the path following.
 *
 * The target point is fixed on the first update: start + distanceMM along the start heading.
 * X (along-track), Y (cross-track) and heading each have their own PID, so the robot also
 * corrects sideways drift and ends on the point, not just the right distance.
 */
@Config // gains / tolerances editable in Dashboard; each new action reads them
public class MoveStraightPIDAction extends Action {
    private static final String TAG = "Drive_MoveStraightPID";

    // Gains start from PurePursuitAction's tuned values. TODO-measure: tune S / D on the robot.
    public static double P_XY = PurePursuitAction.P_XY;
    public static double D_XY = PurePursuitAction.D_XY;
    public static double S_XY = 0.06;
    public static double P_ANGLE = PurePursuitAction.P_ANGLE;
    public static double S_ANGLE = 0.005;

    public static double DEFAULT_TOLERANCE_MM = 10;
    public static double DEFAULT_HEADING_TOLERANCE_DEG = 1;
    public static double DEFAULT_MAX_POWER = 0.7;
    public static double DEFAULT_TIMEOUT_MS = 3000;
    /** Consecutive in-tolerance loops required before finishing, so it doesn't stop mid-overshoot. */
    public static int SETTLE_LOOPS = 3;

    private final DriveTrain driveTrain;
    private final double distanceMM;

    private final PidNav pidX = new PidNav(P_XY, 0, D_XY, S_XY);
    private final PidNav pidY = new PidNav(P_XY, 0, D_XY, S_XY);
    private final PidNav pidAngle = new PidNav(P_ANGLE, 0, 0, S_ANGLE);

    private double toleranceMM = DEFAULT_TOLERANCE_MM;
    private double headingToleranceRad = Math.toRadians(DEFAULT_HEADING_TOLERANCE_DEG);
    private double maxPower = DEFAULT_MAX_POWER;
    private double timeoutMS = DEFAULT_TIMEOUT_MS;

    private Position target;
    private int settledCount = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public MoveStraightPIDAction(DriveTrain driveTrain, double distanceMM) {
        this.driveTrain = driveTrain;
        this.distanceMM = distanceMM;
        this.name = "MoveStraightPID_" + distanceMM;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        Position current = new Position(SharedData.getOdometryWheelIMUPosition());

        if (!hasStarted) {
            double heading = current.getTheta();
            target = new Position(
                    current.getX() + distanceMM * Math.cos(heading),
                    current.getY() + distanceMM * Math.sin(heading),
                    heading);
            timer.reset();
            hasStarted = true;
            KLog.d(TAG, () -> String.format("event=start dist=%.1f start=%s target=%s",
                    distanceMM, current, target));
        }

        if (timer.milliseconds() > timeoutMS) {
            KLog.e(TAG, String.format("event=timeout ms=%.0f current=%s target=%s",
                    timer.milliseconds(), current, target));
            finish();
            return;
        }

        // Robot-frame error, same math as PurePursuitAction.targetPosition()
        Vector currentToTarget = Vector.between(current, target);
        double distanceToTarget = currentToTarget.getLength();
        double directionError = MathFunctions.angleWrapRad(currentToTarget.getHeadingDirection() - current.getTheta());
        double xError = Math.cos(directionError) * distanceToTarget;
        double yError = Math.sin(directionError) * distanceToTarget;
        double angleError = MathFunctions.angleWrapRad(target.getTheta() - current.getTheta());

        if (distanceToTarget < toleranceMM && Math.abs(angleError) < headingToleranceRad) {
            settledCount++;
            driveTrain.setPower(0);
            if (settledCount >= SETTLE_LOOPS) {
                KLog.d(TAG, () -> String.format("event=done ms=%.0f distErr=%.1f angleErrDeg=%.2f current=%s",
                        timer.milliseconds(), distanceToTarget, Math.toDegrees(angleError), current));
                finish();
            }
            return;
        }
        settledCount = 0;

        double powerX = pidX.getPower(xError);
        double powerY = pidY.getPower(yError);
        double powerAngle = pidAngle.getPower(angleError);

        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;
        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;

        // Scale down to maxPower while keeping the wheel ratios (so heading/strafe correction survives)
        double biggest = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        if (biggest > maxPower) {
            double scale = maxPower / biggest;
            fLeftPower *= scale;
            fRightPower *= scale;
            bLeftPower *= scale;
            bRightPower *= scale;
        }

        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format("xErr=%.1f yErr=%.1f angleErrDeg=%.2f pX=%.3f pY=%.3f pA=%.3f",
                    xError, yError, Math.toDegrees(angleError), powerX, powerY, powerAngle));
        }

        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
    }

    private void finish() {
        driveTrain.setPower(0);
        isDone = true;
    }

    @Override
    public void setIsDone(boolean isDone) {
        this.isDone = isDone;
        if (isDone) {
            driveTrain.setPower(0);
        }
    }

    public void setToleranceMM(double toleranceMM) {
        this.toleranceMM = toleranceMM;
    }

    public void setHeadingToleranceDeg(double deg) {
        this.headingToleranceRad = Math.toRadians(deg);
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void setTimeoutMS(double timeoutMS) {
        this.timeoutMS = timeoutMS;
    }

    public Position getTarget() {
        return target;
    }
}
