package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.Raytracer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.KVisionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.BlobSelectionStrategy;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblobbing.BlobUtils;

import java.util.ArrayList;
import java.util.List;

public class MoveToBallAction extends Action {

    private final DriveTrain driveTrain;
    private final KVisionProcessor<List<VisionRecognition>> artifactProcessor;
    private final Raytracer raytracer;
    private final String targetColor;
    private final BlobSelectionStrategy selectionStrategy;

    private PurePursuitAction approachPath;
    private Point detectedBallWorldPos;

    public MoveToBallAction(DriveTrain driveTrain,
                            KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                            Raytracer raytracer,
                            String targetColor,
                            BlobSelectionStrategy selectionStrategy) {
        this.driveTrain = driveTrain;
        this.artifactProcessor = artifactProcessor;
        this.raytracer = raytracer;
        this.targetColor = targetColor;
        this.selectionStrategy = selectionStrategy;
    }

    public MoveToBallAction(DriveTrain driveTrain,
                            KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                            Raytracer raytracer,
                            String targetColor) {
        this(driveTrain, artifactProcessor, raytracer, targetColor,
                BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
    }

    public MoveToBallAction(DriveTrain driveTrain,
                            KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                            Raytracer raytracer) {
        this(driveTrain, artifactProcessor, raytracer, null,
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

        Position robotPose = new Position(SharedData.getOdometryWheelIMUPosition());
        Point worldPos = raytracer.fieldPos(target, VisionConfig.COLOR_BLOB_EDGE_GROW_PX, robotPose);

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
                        raytracer.getCamera().cx, raytracer.getCamera().cy);

            case CLOSEST_TO_ROBOT_WORLD:
                Position robotPos = new Position(SharedData.getOdometryWheelIMUPosition());
                return BlobUtils.findClosestToRobotWorld(candidates,
                        raytracer, robotPos);

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
