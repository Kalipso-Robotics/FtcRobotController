package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;

public class CameraIntrinsics{
    public static int  CAM_WIDTH   = 640;
    public static int  CAM_HEIGHT  = 480;

    private final double fx, fy;
    private final double cx, cy;
    private final double mountAngle;
    private final Vector3d cameraOffset;
    private final double focalLength;
    private final double k1, k2, k3, p1, p2; //distortions
    // Calibrated at 1280x800 (see calibrate_camera.py), scaled to 640x480:
    //   fx,cx *= 0.5   fy,cy *= 0.6   distortion coeffs are dimensionless.
    public static final CameraIntrinsics ARDUCAM = new CameraIntrinsics(
            444.14195, 532.27560,
            350.01860, 212.43984,
            0.045011, -0.059862, 0.000330,
            0.001499, 0.005590,
            Math.toRadians(0),
            new Vector3d(157.548, 236.163, 151.868)
    );


    public CameraIntrinsics(double fx, double fy, double cx, double cy,
                            double k1, double k2, double k3, double p1, double p2,
                            double mountAngle, Vector3d cameraOffset) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.k1 = k1;
        this.k2 = k2;
        this.k3 = k3;
        this.p1 = p1;
        this.p2 = p2;
        this.mountAngle = mountAngle;
        this.cameraOffset = cameraOffset;
        this.focalLength = (fx + fy) / 2;
    }

    public CameraIntrinsics(double fx, double fy, double cx, double cy,
                            double mountAngle, Vector3d cameraOffset) {
        this(fx, fy, cx, cy, 0, 0, 0, 0, 0, mountAngle, cameraOffset);
    }

    public CameraIntrinsics withMount(double mountAngle, Vector3d offset) {
        return new CameraIntrinsics(fx, fy, cx, cy, k1, k2, k3, p1, p2,
                mountAngle, offset);
    }

    /**
     * converts a 2D pixel coordinate (blob) to a 2D world coordinate on the floor
     */
    public Point calculateWorldPos(double pixelX, double pixelY) { // these pixels should be the bottom center of the blob
        // normalize center to middle of image not corner
        double norm_x = pixelX - this.cx;
//        double norm_y = pixelY - this.cy;
        double norm_y = this.cy - pixelY;

        double x_direction = norm_x / this.focalLength;
        double y_direction = norm_y / this.focalLength;

        // the direction of the z in the camera space is always forward (1.0)
        double z_direction = 1.0;

        // account for the pitch of the camera (tilt towards the ground)
        double theta = this.mountAngle;
        double world_y = y_direction * Math.cos(theta) - z_direction * Math.sin(theta);
        double world_z = y_direction * Math.sin(theta) + z_direction * Math.cos(theta);
        double world_x = x_direction;

        if (world_y > 1e-6) {
            return null;
        }

        double t = -cameraOffset.getY() / world_y;

        double floorX = t * world_x;
        double floorZ = t * world_z;

        double xFromRobotCenter = floorX + cameraOffset.getX();
        double zFromRobotCenter = floorZ + cameraOffset.getZ();

        return new Point(xFromRobotCenter, zFromRobotCenter);
    }

    public double getDistanceFromRobot(double pixelX, double pixelY, double robotX, double robotY) {
        Point object = calculateWorldPos(pixelX, pixelY);
        if (object == null) {
            return Double.POSITIVE_INFINITY;
        }
        Point robot = new Point(robotX, robotY);
        return robot.distanceTo(object);
    }

    public double getDistanceFromRobot(VisionRecognition recognition, Point robotPos) {
        Point bottomCenter = recognition.getBottomMiddlePixel();
        return getDistanceFromRobot(bottomCenter.getX(), bottomCenter.getY(),
                robotPos.getX(), robotPos.getY());
    }

    public double getCx() { return cx; }
    public double getCy() { return cy; }
}
