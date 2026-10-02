package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Matrix;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;

/**
 * Where the camera is in the robot frame (x left, y up, z forward, mm): a rotation that maps
 * an OpenCV camera ray into the robot frame, plus the lens position. Immutable.
 *
 * A fixed mount is the constant case. A camera on a moving arm builds a new one per frame
 * from its forward kinematics at the frame's capture time -- no arm maths belongs here.
 */
public class CameraPose {

    /**
     * Pitch (rad) of the Arducam bracket. Fitted against the old bottom-edge floor ray, so it
     * absorbs that method's bias. MUST BE REFIT for the edge-angle ball ray (a sweep over
     * the 2026-09-08 ground truth prefers ~28 deg).
     */
    public static final double ARDUCAM_PITCH_RAD = Math.toRadians(29);
    /** Lens position in the robot frame: x left (negative = right of centre), y up, z forward. */
    public static final Vector3d ARDUCAM_OFFSET = new Vector3d(-157.548, 236.163, 163.470);

    public static final CameraPose ARDUCAM_FIXED =
            fromAngles(ARDUCAM_PITCH_RAD, 0, 0, ARDUCAM_OFFSET);

    /** Camera ray -> robot frame, 3x3. */
    public final Matrix camToRobot;
    /** Lens position in the robot frame, mm. */
    public final Vector3d position;

    public CameraPose(Matrix camToRobot, Vector3d position) {
        this.camToRobot = camToRobot;
        this.position = position;
    }

    /**
     * Builds a row-major 3x3 rotation that maps a normalised OpenCV camera ray
     * ((u-cx)/fx, (v-cy)/fy, 1) -- x right, y down, z forward out of the lens --
     * into the robot frame (x left, y up, z forward).
     *
     * R = Ry(yaw) . Rx(pitchDown) . Rz(roll) . diag(-1,-1,1)
     *
     * diag(-1,-1,1) is the right-to-left / down-to-up flip. Positive pitchDown tilts the lens
     * down, positive yaw turns the lens left, and roll is about the optical axis. All three
     * are in radians.
     */
    public static CameraPose fromAngles(double pitchDown, double yaw, double roll, Vector3d position) {
        double cp = Math.cos(pitchDown), sp = Math.sin(pitchDown);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        double cr = Math.cos(roll), sr = Math.sin(roll);

        Matrix rx = new Matrix(new double[][]{{1, 0, 0}, {0, cp, -sp}, {0, sp, cp}});
        Matrix ry = new Matrix(new double[][]{{cy, 0, sy}, {0, 1, 0}, {-sy, 0, cy}});
        Matrix rz = new Matrix(new double[][]{{cr, -sr, 0}, {sr, cr, 0}, {0, 0, 1}});
        Matrix flip = new Matrix(new double[][]{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}});

        return new CameraPose(ry.multiply(rx).multiply(rz).multiply(flip), position);
    }

    /** Rotates a camera-frame direction into the robot frame. Not normalised. */
    public Vector3d toRobot(double x, double y, double z) {
        return new Vector3d(
                camToRobot.get(0, 0) * x + camToRobot.get(0, 1) * y + camToRobot.get(0, 2) * z,
                camToRobot.get(1, 0) * x + camToRobot.get(1, 1) * y + camToRobot.get(1, 2) * z,
                camToRobot.get(2, 0) * x + camToRobot.get(2, 1) * y + camToRobot.get(2, 2) * z);
    }
}
