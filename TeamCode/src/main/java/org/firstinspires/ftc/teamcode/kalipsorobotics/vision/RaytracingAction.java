package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;
/*
camera_intrinsics:
Image size : 1280 x 800
fx=888.2839 fy=887.1260
cx=700.0372 cy=354.0664
k1=0.045011 k2=-0.059862 k3=0.000330
p1=0.001499 p2=0.005590
*/

public class RaytracingAction extends Action {
    // CONSTANTS (FIND THESE)
    public static double FOCAL_LENGTH = 888; // avg of fx and fy
    public static double CX = 700.0372;
    public static double CY = 354.0664;
    public static double MOUNT_ANGLE = Math.toRadians(0); // angle DOWN to the floor
    Vector3d cameraOffset = new Vector3d(0, 0, 0); // sideways, height, forward

    @Override
    protected void update() {

    }


    /**
     * converts a 2D pixel coordinate (blob) to a 2D world coordinate on the floor
     */
    public Point calculateWorldPos(double pixelX, double pixelY) { // these pixels should be the bottom center of the blob
        // normalize center to middle of image not corner
        double norm_x = pixelX - CX;
        double norm_y = pixelY - CY;

        double x_direction = norm_x / FOCAL_LENGTH;
        double y_direction = norm_y / FOCAL_LENGTH;

        // the direction of the z in the camera space is always forward (1.0)
        double z_direction = 1.0;

        // account for the pitch of the camera (tilt towards the ground)
        double theta = MOUNT_ANGLE;
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
}

