package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;

public class CameraIntrinsics{
    public static int  CAM_WIDTH   = 640;
    public static int  CAM_HEIGHT  = 480;

    private final double fx, fy;
    private final double cx, cy;
    private final double mountAngle;
    private final Vector3d cameraOffset;
    private final double k1, k2, k3, p1, p2; //distortions
    // Calibrated at 1280x800 (see calibrate_camera.py), scaled to 640x480:
    //   fx,cx *= 0.5   fy,cy *= 0.6   distortion coeffs are dimensionless.
    public static final CameraIntrinsics ARDUCAM = new CameraIntrinsics(
            444.14195, 532.27560,
            350.01860, 212.43984,
            0.045011, -0.059862, 0.000330,
            0.001499, 0.005590,
            Math.toRadians(0),
            new Vector3d(-157.548, 236.163, 151.868) // offsets
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
    }

    public CameraIntrinsics(double fx, double fy, double cx, double cy,
                            double mountAngle, Vector3d cameraOffset) {
        this(fx, fy, cx, cy, 0, 0, 0, 0, 0, mountAngle, cameraOffset);
    }

    public CameraIntrinsics withMount(double mountAngle, Vector3d offset) {
        return new CameraIntrinsics(fx, fy, cx, cy, k1, k2, k3, p1, p2,
                mountAngle, offset);
    }

    public Point calculateWorldPos(double pixelX, double pixelY, Position robotPose) {
        Point local = calculateRobotFramePos(pixelX, pixelY);
        if (local == null) return null;
        double cosT = Math.cos(robotPose.getTheta());
        double sinT = Math.sin(robotPose.getTheta());
        double fieldX = robotPose.getX() + local.getY() * cosT - local.getX() * sinT;
        double fieldY = robotPose.getY() + local.getY() * sinT + local.getX() * cosT;
        return new Point(fieldX, fieldY);
    }

    public Point calculateRobotFramePos(double pixelX, double pixelY) {
        double norm_x = this.cx - pixelX;
        // Flipped: image-Y increases downward, but the rest of this math expects
        // a Y-up world (rejection check + t = -height/world_y both need world_y
        // negative for floor pixels). With this sign, bottom-of-image pixels
        // (the floor) actually project; the old `pixelY - cy` rejected them.
        double norm_y = this.cy - pixelY;

        double x_direction = norm_x / this.fx;
        double y_direction = norm_y / this.fy;
        double z_direction = 1.0;

        double theta = this.mountAngle;
        double world_y = y_direction * Math.cos(theta) - z_direction * Math.sin(theta);
        double world_z = y_direction * Math.sin(theta) + z_direction * Math.cos(theta);
        double world_x = x_direction;

        if (world_y > -0.01) {
            return null;
        }

        double t = -cameraOffset.getY() / world_y;

        double floorX = t * world_x;
        double floorZ = t * world_z;

        return new Point(floorX + cameraOffset.getX(), floorZ + cameraOffset.getZ());
    }

    public double getDistanceFromRobot(double pixelX, double pixelY, Position robotPose) {
        Point object = calculateWorldPos(pixelX, pixelY, robotPose);
        if (object == null) {
            return Double.POSITIVE_INFINITY;
        }
        return robotPose.toPoint().distanceTo(object);
    }

    public double getDistanceFromRobot(VisionRecognition recognition, Position robotPose) {
        Point bottomCenter = recognition.getBottomMiddlePixel();
        return getDistanceFromRobot(bottomCenter.getX(), bottomCenter.getY(), robotPose);
    }

    /**
     * Ranges by known real-world object size instead of floor-plane intersection.
     * Unlike calculateRobotFramePos, this works for objects not resting on the floor
     * (elevated, stacked, mid-air) since it never assumes a floor plane.
     *
     * @param objectDiameterMM real-world diameter of the (assumed circular) object,
     *                         e.g. KColorBlobProcessor.getObjectDiameterMM()
     */
    public Point calculateRobotFramePosFromSize(VisionRecognition recognition, double objectDiameterMM) {
        double pixelDiameter = (recognition.getWidth() + recognition.getHeight()) / 2.0;
        if (pixelDiameter <= 0) return null;
        double range = (objectDiameterMM * this.fx) / pixelDiameter; // depth along camera Z-axis

        double norm_x = this.cx - recognition.center.getX();
        double norm_y = this.cy - recognition.center.getY();
        double x_direction = norm_x / this.fx;
        double y_direction = norm_y / this.fy;
        double z_direction = 1.0;

        double theta = this.mountAngle;
        double world_y = y_direction * Math.cos(theta) - z_direction * Math.sin(theta);
        double world_z = y_direction * Math.sin(theta) + z_direction * Math.cos(theta);
        double world_x = x_direction;

        double floorX = range * world_x;
        double floorZ = range * world_z;

        return new Point(floorX + cameraOffset.getX(), floorZ + cameraOffset.getZ());
    }

    public Point calculateWorldPosFromSize(VisionRecognition recognition, double objectDiameterMM, Position robotPose) {
        Point local = calculateRobotFramePosFromSize(recognition, objectDiameterMM);
        if (local == null) return null;
        double cosT = Math.cos(robotPose.getTheta());
        double sinT = Math.sin(robotPose.getTheta());
        double fieldX = robotPose.getX() + local.getY() * cosT - local.getX() * sinT;
        double fieldY = robotPose.getY() + local.getY() * sinT + local.getX() * cosT;
        return new Point(fieldX, fieldY);
    }

    public double getDistanceFromRobotBySize(VisionRecognition recognition, double objectDiameterMM, Position robotPose) {
        Point object = calculateWorldPosFromSize(recognition, objectDiameterMM, robotPose);
        if (object == null) {
            return Double.POSITIVE_INFINITY;
        }
        return robotPose.toPoint().distanceTo(object);
    }

    public double getCx() { return cx; }
    public double getCy() { return cy; }
}
