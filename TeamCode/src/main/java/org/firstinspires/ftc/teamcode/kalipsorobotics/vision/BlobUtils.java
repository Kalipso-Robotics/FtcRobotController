package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.List;

public class BlobUtils {

    public static DetectedBlob findClosestToCameraCenter(List<DetectedBlob> blobs, double cx, double cy) {
        if (blobs == null || blobs.isEmpty()) {
            return null;
        }

        DetectedBlob closest = null;
        double minDistance = Double.POSITIVE_INFINITY;

        for (DetectedBlob blob : blobs) {
            double distance = blob.center.distanceTo(new Point(cx, cy));
            if (distance < minDistance) {
                minDistance = distance;
                closest = blob;
            }
        }
        return closest;
    }

    public static DetectedBlob findClosestToRobotWorld(List<DetectedBlob> blobs,
                                                        CameraIntrinsics intrinsics,
                                                        Point robotPos) {
        if (blobs == null || blobs.isEmpty()) {
            return null;
        }

        DetectedBlob closest = null;
        double minDistance = Double.POSITIVE_INFINITY;

        for (DetectedBlob blob : blobs) {
            double distance = intrinsics.getDistanceFromRobot(blob, robotPos);
            if (distance < minDistance) {
                minDistance = distance;
                closest = blob;
            }
        }

        return closest;
    }

    public static DetectedBlob findMostCircular(List<DetectedBlob> blobs) {
        if (blobs == null || blobs.isEmpty()) {
            return null;
        }

        DetectedBlob mostCircular = blobs.get(0);
        for (int i = 1; i < blobs.size(); i++) {
            if (blobs.get(i).circularity > mostCircular.circularity) {
                mostCircular = blobs.get(i);
            }
        }

        return mostCircular;
    }

    public static DetectedBlob findLargestByArea(List<DetectedBlob> blobs) {
        if (blobs == null || blobs.isEmpty()) {
            return null;
        }
        return blobs.get(0);
    }
}
