package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.CameraMountConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.CameraGimbal;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 2-servo pan/tilt mount for the AprilTag camera, letting it sweep across multiple tags
 * instead of staring in one fixed direction. Mirrors Turret's singleton/instance pattern.
 *
 * Pan (yaw, left/right) is the only axis that feeds into AprilTagDetectionAction's planar
 * (x, y, theta) localization math - see getMountRelCamPos(). Tilt (pitch, up/down) is
 * reported for aiming/telemetry but intentionally does NOT affect that transform: the
 * chain only ever works in the horizontal plane, so tilt only matters as long as the
 * camera's optical center stays essentially at the pan/tilt pivot (PIVOT_REL_CAM_POS ~ 0).
 * If the built mount ends up with the lens meaningfully offset from the pivot, tilt would
 * need to become a real 3D rotation here - out of scope until that's true.
 */
public class CameraMount implements CameraGimbal {

    private static CameraMount single_instance = null;

    private OpModeUtilities opModeUtilities;

    private KServo panServo;
    private KServo tiltServo;

    private CameraMount(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        resetHardwareMap(opModeUtilities, opModeUtilities.getHardwareMap(), this);
    }

    public static synchronized CameraMount getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new CameraMount(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities, opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(OpModeUtilities opModeUtilities, HardwareMap hardwareMap, CameraMount cameraMount) {
        cameraMount.opModeUtilities = opModeUtilities;
        Servo pan = hardwareMap.servo.get("cameraPan");
        Servo tilt = hardwareMap.servo.get("cameraTilt");
        cameraMount.panServo = new KServo(pan);
        cameraMount.tiltServo = new KServo(tilt);
    }

    public void setPanAngleRad(double angleRad) {
        panServo.setTargetPosition(angleToServoPosition(angleRad, CameraMountConfig.PAN_MIN_ANGLE_DEG, CameraMountConfig.PAN_MAX_ANGLE_DEG, CameraMountConfig.PAN_FLIP_DIRECTION));
    }

    public void setTiltAngleRad(double angleRad) {
        tiltServo.setTargetPosition(angleToServoPosition(angleRad, CameraMountConfig.TILT_MIN_ANGLE_DEG, CameraMountConfig.TILT_MAX_ANGLE_DEG, CameraMountConfig.TILT_FLIP_DIRECTION));
    }

    public double getCurrentPanAngleRad() {
        return servoPositionToAngleRad(panServo.getPosition(), CameraMountConfig.PAN_MIN_ANGLE_DEG, CameraMountConfig.PAN_MAX_ANGLE_DEG, CameraMountConfig.PAN_FLIP_DIRECTION);
    }

    public double getCurrentTiltAngleRad() {
        return servoPositionToAngleRad(tiltServo.getPosition(), CameraMountConfig.TILT_MIN_ANGLE_DEG, CameraMountConfig.TILT_MAX_ANGLE_DEG, CameraMountConfig.TILT_FLIP_DIRECTION);
    }

    /**
     * Camera's position in the turret's local frame, given the pivot's fixed offset from
     * the turret plus the camera's fixed offset from the pivot, rotated by the live pan angle.
     */
    @Override
    public Position getMountRelCamPos() {
        Position pivotFrame = new Position(
                CameraMountConfig.TURRET_REL_GIMBAL_PIVOT_POS.getX(),
                CameraMountConfig.TURRET_REL_GIMBAL_PIVOT_POS.getY(),
                -getCurrentPanAngleRad());
        return CameraMountConfig.PIVOT_REL_CAM_POS.toNewFrame(pivotFrame);
    }

    private static double angleToServoPosition(double angleRad, double minAngleDeg, double maxAngleDeg, boolean flipDirection) {
        double angleDeg = Math.toDegrees(angleRad);
        double t = (angleDeg - minAngleDeg) / (maxAngleDeg - minAngleDeg);
        t = flipDirection ? 1 - t : t;
        return MathFunctions.clamp(t, 0, 1);
    }

    private static double servoPositionToAngleRad(double position, double minAngleDeg, double maxAngleDeg, boolean flipDirection) {
        double t = flipDirection ? 1 - position : position;
        double angleDeg = minAngleDeg + t * (maxAngleDeg - minAngleDeg);
        return Math.toRadians(angleDeg);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
