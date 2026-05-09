package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.ArtifactDetectionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BlobSelectionStrategy;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BlobUtils;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.DetectedBlob;

import java.util.List;

public class MoveToBallAction extends Action {

    private final DriveTrain driveTrain;
    private final ArtifactDetectionProcessor artifactProcessor;
    private final CameraIntrinsics cameraIntrinsics;
    private final String targetColor;
    private final BlobSelectionStrategy selectionStrategy;

    private PurePursuitAction approachPath;
    private Point detectedBallWorldPos;

    public MoveToBallAction(DriveTrain driveTrain,
                              ArtifactDetectionProcessor artifactProcessor,
                              CameraIntrinsics cameraIntrinsics,
                              String targetColor,
                              BlobSelectionStrategy selectionStrategy) {
        this.driveTrain = driveTrain;
        this.artifactProcessor = artifactProcessor;
        this.cameraIntrinsics = cameraIntrinsics;
        this.targetColor = targetColor;
        this.selectionStrategy = selectionStrategy;
    }

    public MoveToBallAction(DriveTrain driveTrain,
                              ArtifactDetectionProcessor artifactProcessor,
                              CameraIntrinsics cameraIntrinsics,
                              String targetColor) {
        this(driveTrain, artifactProcessor, cameraIntrinsics, targetColor,
                BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
    }

    public MoveToBallAction(DriveTrain driveTrain,
                              ArtifactDetectionProcessor artifactProcessor,
                              CameraIntrinsics cameraIntrinsics) {
        this(driveTrain, artifactProcessor, cameraIntrinsics, null,
                BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
    }

    @Override
    protected void update() {
        if (isDone) return;

        DetectedBlob targetBlob = selectBlob();
        if (targetBlob == null) {
            String colorFilter = targetColor != null ? targetColor + " " : "";
            KLog.d("MoveToBall", () -> String.format("No %sball detected", colorFilter));
            isDone = true;
            return;
        }

        Point bottomCenter = targetBlob.getBottomMiddlePixel();
        Point worldPos = cameraIntrinsics.calculateWorldPos(bottomCenter.getX(), bottomCenter.getY());

        if (worldPos == null) {
            KLog.d("MoveToBall", "Failed to convert blob to world coordinates");
            isDone = true;
            return;
        }

        detectedBallWorldPos = worldPos;
        String colorLabel = targetBlob.colorLabel != null ? targetBlob.colorLabel + " " : "";
        KLog.d("MoveToBall", () -> String.format("%sball detected (%s) at world position: (%.1f, %.1f)",
                colorLabel, selectionStrategy, worldPos.getX(), worldPos.getY()));

        approachPath = createApproachPath(worldPos);
        isDone = true;
    }

    private DetectedBlob selectBlob() {
        List<DetectedBlob> allBlobs = artifactProcessor.getLatestResult();
        if (allBlobs == null || allBlobs.isEmpty()) {
            return null;
        }

        // Filter by color if specified
        List<DetectedBlob> candidateBlobs = filterByColor(allBlobs, targetColor);
        if (candidateBlobs.isEmpty()) {
            return null;
        }

        // Select blob using strategy
        switch (selectionStrategy) {
            case LARGEST_AREA:
                return BlobUtils.findLargestByArea(candidateBlobs);

            case CLOSEST_TO_CAMERA_CENTER:
                return BlobUtils.findClosestToCameraCenter(candidateBlobs,
                        cameraIntrinsics.getCx(), cameraIntrinsics.getCy());

            case CLOSEST_TO_ROBOT_WORLD:
                Position robotPos = new Position(SharedData.getOdometryWheelIMUPosition());
                return BlobUtils.findClosestToRobotWorld(candidateBlobs,
                        cameraIntrinsics, robotPos.toPoint());

            case MOST_CIRCULAR:
                return BlobUtils.findMostCircular(candidateBlobs);

            default:
                return candidateBlobs.get(0);
        }
    }

    private List<DetectedBlob> filterByColor(List<DetectedBlob> blobs, String colorLabel) {
        if (colorLabel == null) {
            return blobs;
        }

        List<DetectedBlob> filtered = new java.util.ArrayList<>();
        for (DetectedBlob blob : blobs) {
            if (colorLabel.equals(blob.colorLabel)) {
                filtered.add(blob);
            }
        }
        return filtered;
    }

    protected PurePursuitAction createApproachPath(Point ballWorldPos) {
        Position currentPos = new Position(SharedData.getOdometryWheelIMUPosition());
        PurePursuitAction pursuit = new PurePursuitAction(driveTrain);

        pursuit.addPoint(ballWorldPos.getX(), ballWorldPos.getY(),
                Math.toDegrees(currentPos.getTheta()));
        pursuit.setName("MoveToBall" + targetColor);
        pursuit.setFinalSearchRadiusMM(100);
        pursuit.setLookAheadRadius(125);
        pursuit.setMaxTimeOutMS(5000);

        return pursuit;
    }

    public PurePursuitAction getApproachPath() { return approachPath; }
    public Point getDetectedBallWorldPos() { return detectedBallWorldPos; }
    public boolean hasDetectedBall() { return approachPath != null; }
}
