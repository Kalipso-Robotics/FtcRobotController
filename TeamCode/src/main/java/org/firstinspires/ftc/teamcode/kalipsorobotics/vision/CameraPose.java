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
     * OpenCV camera axes (right, down, forward) -> robot axes (left, up, forward).
     * A true rotation, not a mirror: det = +1, because both triads are right-handed.
     */
    public static final Matrix CV_TO_ROBOT =
            new Matrix(new double[][]{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}});

    // Declared first: the fixed poses below are built during class init and need it.
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

    public static Matrix rotX(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Matrix(new double[][]{{1, 0, 0}, {0, c, -s}, {0, s, c}});
    }

    public static Matrix rotY(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Matrix(new double[][]{{c, 0, s}, {0, 1, 0}, {-s, 0, c}});
    }

    public static Matrix rotZ(double a) {
        double c = Math.cos(a), s = Math.sin(a);
        return new Matrix(new double[][]{{c, -s, 0}, {s, c, 0}, {0, 0, 1}});
    }

    /**
     * Rotation mapping a normalised OpenCV ray ((u-cx)/fx, (v-cy)/fy, 1) into the robot frame.
     * Read right to left: flip to robot axes, roll about the optical axis, pitch the lens down,
     * then yaw it left. Angles in radians.
     */
    public static CameraPose fromAngles(double pitchDown, double yaw, double roll, Vector3d position) {
        Matrix camToRobot = rotY(yaw).multiply(rotX(pitchDown)).multiply(rotZ(roll)).multiply(CV_TO_ROBOT);
        return new CameraPose(camToRobot, position);
    }

    /** Rotates a camera-frame direction into the robot frame. Not normalised. */
    public Vector3d toRobot(double x, double y, double z) {
        return new Vector3d(
                camToRobot.get(0, 0) * x + camToRobot.get(0, 1) * y + camToRobot.get(0, 2) * z,
                camToRobot.get(1, 0) * x + camToRobot.get(1, 1) * y + camToRobot.get(1, 2) * z,
                camToRobot.get(2, 0) * x + camToRobot.get(2, 1) * y + camToRobot.get(2, 2) * z);
    }
}
