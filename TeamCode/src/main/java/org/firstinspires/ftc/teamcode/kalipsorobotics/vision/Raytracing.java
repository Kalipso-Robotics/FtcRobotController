package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Vector3d;

public class Raytracing {
    // CONSTANTS (FIND THESE)
    public static double FOCAL_LENGTH = 640;
    public static double IMAGE_WIDTH = 640;
    public static int IMAGE_HEIGHT = 480;
    public static double MOUNT_ANGLE = Math.toRadians(0); // angle DOWN to the floor

    Vector3d cameraOffset = new Vector3d(0, 0, 0); // sideways, height, forward

    /**
     * converts a 2D pixel coordinate to a 2D world coordinate on the floor
     */
    public Point calculateWorldPos(double pixelX, double pixelY) { // these pixels should be the bottom center of the blob
        // normalize center to middle of image not corner
        double norm_x = pixelX - (IMAGE_WIDTH / 2.0);
        double norm_y = pixelY - (IMAGE_HEIGHT / 2.0);

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

