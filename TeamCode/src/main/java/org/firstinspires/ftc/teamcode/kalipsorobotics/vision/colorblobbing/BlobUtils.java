package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BallDetection;

import java.util.List;

/**
 * Selection helpers that work on any KVisionRecognition list — color blobs, TFLite detections,
 * or anything else a KVisionProcessor produces.
 */
public class BlobUtils {

    public static BallDetection findClosestToCameraCenter(List<BallDetection> recognitions,
                                                          double cx, double cy) {
        if (recognitions == null || recognitions.isEmpty()) return null;

        BallDetection closest = null;
        double minDistance = Double.POSITIVE_INFINITY;
        for (BallDetection recognition : recognitions) {
            double distance = recognition.center.distanceTo(new Point(cx, cy));
            if (distance < minDistance) {
                minDistance = distance;
                closest = recognition;
            }
        }
        return closest;
    }

    public static BallDetection findClosestToRobotWorld(List<BallDetection> recognitions,
                                                        CameraIntrinsics intrinsics,
                                                        Position robotPose) {
        if (recognitions == null || recognitions.isEmpty()) return null;

        BallDetection closest = null;
        double minDistance = Double.POSITIVE_INFINITY;
        for (BallDetection recognition : recognitions) {
            double distance = intrinsics.getDistanceFromRobot(recognition, robotPose);
            if (distance < minDistance) {
                minDistance = distance;
                closest = recognition;
            }
        }
        return closest;
    }

    public static BallDetection findMostCircular(List<BallDetection> recognitions) {
        if (recognitions == null || recognitions.isEmpty()) return null;

        BallDetection mostCircular = recognitions.get(0);
        for (int i = 1; i < recognitions.size(); i++) {
            if (recognitions.get(i).getCircularity() > mostCircular.getCircularity()) {
                mostCircular = recognitions.get(i);
            }
        }
        return mostCircular;
    }

    /** The KColorBlobProcessor sorts largest-first, so the first entry is the largest. */
    public static BallDetection findLargestByArea(List<BallDetection> recognitions) {
        if (recognitions == null || recognitions.isEmpty()) return null;
        BallDetection largest = recognitions.get(0);
        for (int i = 1; i < recognitions.size(); i++) {
            if (recognitions.get(i).getArea() > largest.getArea()) {
                largest = recognitions.get(i);
            }
        }
        return largest;
    }
}
