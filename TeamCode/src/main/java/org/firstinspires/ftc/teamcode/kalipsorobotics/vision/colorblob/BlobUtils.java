package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;

import java.util.List;

/**
 * Selection helpers that work on any KVisionRecognition list — color blobs, TFLite detections,
 * or anything else a KVisionProcessor produces.
 */
public class BlobUtils {

    public static VisionRecognition findClosestToCameraCenter(List<VisionRecognition> recognitions,
                                                              double cx, double cy) {
        if (recognitions == null || recognitions.isEmpty()) return null;

        VisionRecognition closest = null;
        double minDistance = Double.POSITIVE_INFINITY;
        for (VisionRecognition recognition : recognitions) {
            double distance = recognition.center.distanceTo(new Point(cx, cy));
            if (distance < minDistance) {
                minDistance = distance;
                closest = recognition;
            }
        }
        return closest;
    }

    public static VisionRecognition findClosestToRobotWorld(List<VisionRecognition> recognitions,
                                                            CameraIntrinsics intrinsics,
                                                            Point robotPos) {
        if (recognitions == null || recognitions.isEmpty()) return null;

        VisionRecognition closest = null;
        double minDistance = Double.POSITIVE_INFINITY;
        for (VisionRecognition recognition : recognitions) {
            double distance = intrinsics.getDistanceFromRobot(recognition, robotPos);
            if (distance < minDistance) {
                minDistance = distance;
                closest = recognition;
            }
        }
        return closest;
    }

    public static VisionRecognition findMostCircular(List<VisionRecognition> recognitions) {
        if (recognitions == null || recognitions.isEmpty()) return null;

        VisionRecognition mostCircular = recognitions.get(0);
        for (int i = 1; i < recognitions.size(); i++) {
            if (recognitions.get(i).getCircularity() > mostCircular.getCircularity()) {
                mostCircular = recognitions.get(i);
            }
        }
        return mostCircular;
    }

    /** The KColorBlobProcessor sorts largest-first, so the first entry is the largest. */
    public static VisionRecognition findLargestByArea(List<VisionRecognition> recognitions) {
        if (recognitions == null || recognitions.isEmpty()) return null;
        VisionRecognition largest = recognitions.get(0);
        for (int i = 1; i < recognitions.size(); i++) {
            if (recognitions.get(i).getArea() > largest.getArea()) {
                largest = recognitions.get(i);
            }
        }
        return largest;
    }
}
