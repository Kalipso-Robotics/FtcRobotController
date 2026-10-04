package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.Locale;

/** Result of CameraIntrinsics.estimateBall. Immutable. Lengths in mm. */
public class BallEstimate {

    /** Ball centre in the robot frame: x = left, y = forward. */
    public final Point robotPos;
    /** Camera-frame depth from the ray/plane intersection (the ray parameter t). */
    public final double rayDepthMM;
    /** Depth implied by the ball's vertical angular size (uses fy). */
    public final double sizeDepthVMM;
    /** Depth implied by the ball's horizontal angular size (uses fx, known ~16% low: log only). */
    public final double sizeDepthHMM;

    // Intermediates, for calibration and telemetry.
    /** Centre ray angles: horizontal (psi) and vertical (theta), rad, from the optical axis. */
    public final double psiRad, thetaRad;
    /** Angular radius of the ball per axis, rad. */
    public final double deltaXRad, deltaYRad;
    /** The centre ray rotated into the robot frame (x left, y up, z forward), not normalised. */
    public final Vector3d rayRobot;

    private final double maxDisagreement;

    BallEstimate(Point robotPos, double rayDepthMM, double sizeDepthVMM, double sizeDepthHMM,
                 double psiRad, double thetaRad, double deltaXRad, double deltaYRad,
                 Vector3d rayRobot, double maxDisagreement) {
        this.robotPos = robotPos;
        this.rayDepthMM = rayDepthMM;
        this.sizeDepthVMM = sizeDepthVMM;
        this.sizeDepthHMM = sizeDepthHMM;
        this.psiRad = psiRad;
        this.thetaRad = thetaRad;
        this.deltaXRad = deltaXRad;
        this.deltaYRad = deltaYRad;
        this.rayRobot = rayRobot;
        this.maxDisagreement = maxDisagreement;
    }

    /** (sizeDepthV - rayDepth) / rayDepth. Positive: the ball looks farther by size than by ray. */
    public double rangeDisagreement() {
        return (sizeDepthVMM - rayDepthMM) / rayDepthMM;
    }

    /** False for a ball off the floor, a merged/occluded blob or the wrong radius. */
    public boolean isConsistent() {
        return Math.abs(rangeDisagreement()) <= maxDisagreement;
    }

    /** Field position, given the robot pose at the moment the frame was captured. */
    public Point fieldPos(Position robotPoseAtCapture) {
        return CameraIntrinsics.toField(robotPos, robotPoseAtCapture);
    }

    /**
     * Field position using the odometry pose at the frame's capture time (System.nanoTime()
     * domain). Null if that pose is older than the history, i.e. too stale to project with.
     */
    public Point fieldPosAt(long captureNanos) {
        Position pose = SharedData.getOdometryWheelIMUPositionAt(captureNanos);
        return pose == null ? null : fieldPos(pose);
    }

    @Override
    public String toString() {
        return String.format(Locale.US,
                "BallEstimate{robot=(%.1f,%.1f) t=%.1f sizeV=%.1f sizeH=%.1f disagree=%+.1f%% ok=%b"
                        + " psi=%.2f theta=%.2f dX=%.2f dY=%.2f deg}",
                robotPos.getX(), robotPos.getY(), rayDepthMM, sizeDepthVMM, sizeDepthHMM,
                100 * rangeDisagreement(), isConsistent(),
                Math.toDegrees(psiRad), Math.toDegrees(thetaRad),
                Math.toDegrees(deltaXRad), Math.toDegrees(deltaYRad));
    }
}
