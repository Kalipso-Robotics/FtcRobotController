package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;

import java.util.Locale;

/** Result of CameraIntrinsics.estimateBall. Immutable. Lengths in mm. */
public class BallEstimate {

    /** Ball centre in the robot frame: x = left, y = forward. */
    public final Point robotPos;
    /** Camera-frame depth from the ray/plane intersection. */
    public final double rayDepthMM;
    /** Depth implied by the ball's vertical angular size (uses fy). */
    public final double sizeDepthVMM;
    /** Depth implied by the ball's horizontal angular size (uses fx, known ~16% low: log only). */
    public final double sizeDepthHMM;

    private final double maxDisagreement;

    BallEstimate(Point robotPos, double rayDepthMM, double sizeDepthVMM, double sizeDepthHMM,
                 double maxDisagreement) {
        this.robotPos = robotPos;
        this.rayDepthMM = rayDepthMM;
        this.sizeDepthVMM = sizeDepthVMM;
        this.sizeDepthHMM = sizeDepthHMM;
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

    @Override
    public String toString() {
        return String.format(Locale.US,
                "BallEstimate{robot=(%.1f,%.1f) rayZ=%.1f sizeV=%.1f sizeH=%.1f disagree=%+.1f%% ok=%b}",
                robotPos.getX(), robotPos.getY(), rayDepthMM, sizeDepthVMM, sizeDepthHMM,
                100 * rangeDisagreement(), isConsistent());
    }
}
