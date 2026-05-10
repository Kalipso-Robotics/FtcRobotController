package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.KVisionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob.BlobSelectionStrategy;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob.BlobUtils;

import java.util.ArrayList;
import java.util.List;

public class MoveToBallAction extends Action {

    private final DriveTrain driveTrain;
    private final KVisionProcessor<List<VisionRecognition>> artifactProcessor;
    private final CameraIntrinsics cameraIntrinsics;
    private final String targetColor;
    private final BlobSelectionStrategy selectionStrategy;

    private PurePursuitAction approachPath;
    private Point detectedBallWorldPos;

    public MoveToBallAction(DriveTrain driveTrain,
                            KVisionProcessor<List<VisionRecognition>> artifactProcessor,
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
                            KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                            CameraIntrinsics cameraIntrinsics,
                            String targetColor) {
        this(driveTrain, artifactProcessor, cameraIntrinsics, targetColor,
                BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
    }

    public MoveToBallAction(DriveTrain driveTrain,
                            KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                            CameraIntrinsics cameraIntrinsics) {
        this(driveTrain, artifactProcessor, cameraIntrinsics, null,
                BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
    }

    @Override
    protected void update() {
        if (isDone) return;

        VisionRecognition target = selectRecognition();
        if (target == null) {
            String colorFilter = targetColor != null ? targetColor + " " : "";
            KLog.d("MoveToBall", () -> String.format("No %sball detected", colorFilter));
            isDone = true;
            return;
        }

        Point bottomCenter = target.getBottomMiddlePixel();
        Point worldPos = cameraIntrinsics.calculateWorldPos(bottomCenter.getX(), bottomCenter.getY());

        if (worldPos == null) {
            KLog.d("MoveToBall", "Failed to convert detection to world coordinates");
            isDone = true;
            return;
        }

        detectedBallWorldPos = worldPos;
        KLog.d("MoveToBall", () -> String.format("%s detected (%s) at world position: (%.1f, %.1f)",
                target.label, selectionStrategy, worldPos.getX(), worldPos.getY()));

        approachPath = createApproachPath(worldPos);
        isDone = true;
    }

    private VisionRecognition selectRecognition() {
        List<VisionRecognition> all = artifactProcessor.getLatestResult();
        if (all == null || all.isEmpty()) return null;

        List<VisionRecognition> candidates = filterByLabel(all, targetColor);
        if (candidates.isEmpty()) return null;

        switch (selectionStrategy) {
            case LARGEST_AREA:
                return BlobUtils.findLargestByArea(candidates);

            case CLOSEST_TO_CAMERA_CENTER:
                return BlobUtils.findClosestToCameraCenter(candidates,
                        cameraIntrinsics.getCx(), cameraIntrinsics.getCy());

            case CLOSEST_TO_ROBOT_WORLD:
                Position robotPos = new Position(SharedData.getOdometryWheelIMUPosition());
                return BlobUtils.findClosestToRobotWorld(candidates,
                        cameraIntrinsics, robotPos.toPoint());

            case MOST_CIRCULAR:
                return BlobUtils.findMostCircular(candidates);

            default:
                return candidates.get(0);
        }
    }

    private List<VisionRecognition> filterByLabel(List<VisionRecognition> recognitions, String label) {
        if (label == null) return recognitions;

        List<VisionRecognition> filtered = new ArrayList<>();
        for (VisionRecognition recognition : recognitions) {
            if (label.equals(recognition.label)) filtered.add(recognition);
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
