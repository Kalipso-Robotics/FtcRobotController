package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.KVisionProcessor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob.BlobSelectionStrategy;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.colorblob.BlobUtils;

import java.util.ArrayList;
import java.util.List;

public class VisionRoundTripAction extends RoundTripAction {
    private final boolean useVision;
    private final KVisionProcessor<List<VisionRecognition>> artifactProcessor;
    private final CameraIntrinsics cameraIntrinsics;
    private final String targetBallColor;
    private final BlobSelectionStrategy selectionStrategy;

    // When set, vision is deferred until the robot is within lookoutRadiusMM of this point.
    // The path always drives through the lookout point first; vision decides what comes next.
    private final Point visionLookoutPoint;
    private final double lookoutRadiusMM;

    private boolean visionProcessed = false;
    private Point detectedBallWorldPos;

    private VisionRoundTripAction(Builder builder) {
        super(builder.opModeUtilities, builder.driveTrain, builder.turretAutoAlign,
                builder.shooter, builder.stopper, builder.intake,
                builder.targetPoint, builder.launchPoint, builder.waitForShooterReadyMS,
                builder.shouldRunIntake, builder.shouldDependOnFlywheel);

        this.useVision = builder.useVision;
        this.artifactProcessor = builder.artifactProcessor;
        this.cameraIntrinsics = builder.cameraIntrinsics;
        this.targetBallColor = builder.targetBallColor;
        this.selectionStrategy = builder.selectionStrategy;
        this.visionLookoutPoint = builder.visionLookoutPoint;
        this.lookoutRadiusMM = builder.lookoutRadiusMM;

        if (useVision && (artifactProcessor == null || cameraIntrinsics == null)) {
            throw new IllegalArgumentException(
                    "Vision mode requires: artifactProcessor and cameraIntrinsics");
        }
    }

    public static class Builder {
        private final OpModeUtilities opModeUtilities;
        private final DriveTrain driveTrain;
        private final TurretAutoAlign turretAutoAlign;
        private final Shooter shooter;
        private final Stopper stopper;
        private final Intake intake;

        private Point targetPoint = Shooter.TARGET_POINT;
        private Point launchPoint = new Point(0, 0);
        private double waitForShooterReadyMS = 0;
        private boolean shouldRunIntake = true;
        private boolean shouldDependOnFlywheel = false;

        private boolean useVision = false;
        private KVisionProcessor<List<VisionRecognition>> artifactProcessor;
        private CameraIntrinsics cameraIntrinsics;
        private String targetBallColor;
        private BlobSelectionStrategy selectionStrategy = BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER;

        private Point visionLookoutPoint = null;
        private double lookoutRadiusMM = 250;

        public Builder(OpModeUtilities opModeUtilities,
                       DriveTrain driveTrain,
                       TurretAutoAlign turretAutoAlign,
                       Shooter shooter,
                       Stopper stopper,
                       Intake intake) {
            this.opModeUtilities = opModeUtilities;
            this.driveTrain = driveTrain;
            this.turretAutoAlign = turretAutoAlign;
            this.shooter = shooter;
            this.stopper = stopper;
            this.intake = intake;
        }

        public Builder setTargetPoint(Point targetPoint) {
            this.targetPoint = targetPoint;
            return this;
        }

        public Builder setLaunchPoint(Point launchPoint) {
            this.launchPoint = launchPoint;
            return this;
        }

        public Builder setWaitForShooterReadyMS(double waitMS) {
            this.waitForShooterReadyMS = waitMS;
            return this;
        }

        public Builder setShouldRunIntake(boolean shouldRunIntake) {
            this.shouldRunIntake = shouldRunIntake;
            return this;
        }

        public Builder setShouldDependOnFlywheel(boolean shouldDependOnFlywheel) {
            this.shouldDependOnFlywheel = shouldDependOnFlywheel;
            return this;
        }

        /**
         * Set a mandatory lookout point. The robot drives to this point first.
         * Vision is processed once the robot is within radiusMM of the lookout.
         * If a ball is seen → navigate to it. If not → continue on fallback waypoints.
         */
        public Builder setVisionLookoutPoint(Point lookoutPoint, double radiusMM) {
            this.visionLookoutPoint = lookoutPoint;
            this.lookoutRadiusMM = radiusMM;
            return this;
        }

        public Builder setVisionLookoutPoint(Point lookoutPoint) {
            return setVisionLookoutPoint(lookoutPoint, 250);
        }

        /**
         * Enable vision-guided ball selection.
         *
         * @param artifactProcessor Any KVisionProcessor that produces List<KVisionRecognition>
         *                          — works for color blob processors AND TFLite detectors.
         * @param cameraIntrinsics  Camera calibration for pixel-to-world conversion.
         * @param targetBallColor   Filter by recognition.label (e.g. "Purple", "Green").
         *                          Pass null to accept any color/label (use for TFLite-single-class).
         * @param selectionStrategy Which detection to pick when multiple are seen.
         */
        public Builder enableVision(KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                                    CameraIntrinsics cameraIntrinsics,
                                    String targetBallColor,
                                    BlobSelectionStrategy selectionStrategy) {
            this.useVision = true;
            this.artifactProcessor = artifactProcessor;
            this.cameraIntrinsics = cameraIntrinsics;
            this.targetBallColor = targetBallColor;
            this.selectionStrategy = selectionStrategy;
            return this;
        }

        public Builder enableVision(KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                                    CameraIntrinsics cameraIntrinsics,
                                    String targetBallColor) {
            return enableVision(artifactProcessor, cameraIntrinsics, targetBallColor,
                    BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
        }

        public Builder enableVision(KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                                    CameraIntrinsics cameraIntrinsics) {
            return enableVision(artifactProcessor, cameraIntrinsics, null,
                    BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
        }

        public Builder enableVision(KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                                    CameraIntrinsics cameraIntrinsics,
                                    BlobSelectionStrategy selectionStrategy) {
            return enableVision(artifactProcessor, cameraIntrinsics, null, selectionStrategy);
        }

        public VisionRoundTripAction build() {
            return new VisionRoundTripAction(this);
        }
    }

    @Override
    protected void beforeUpdate() {
        if (useVision && !visionProcessed) {
            if (visionLookoutPoint != null) {
                Position currentPos = SharedData.getOdometryWheelIMUPosition();
                double dist = currentPos.toPoint().distanceTo(visionLookoutPoint);
                if (dist < lookoutRadiusMM) {
                    KLog.d("VisionRoundTrip", () -> String.format("[%s] At lookout (dist=%.0fmm) - processing vision", getName(), dist));
                    processVision();
                    visionProcessed = true;
                }
            } else if (!hasStarted) {
                processVision();
                visionProcessed = true;
            }
        }
        super.beforeUpdate();
    }

    private void processVision() {
        VisionRecognition target = getTargetRecognition();
        if (target == null) {
            String colorFilter = targetBallColor != null ? targetBallColor + " " : "";
            KLog.d("VisionRoundTrip", () -> String.format("[%s] No %sball detected - continuing on fallback waypoints",
                    getName(), colorFilter));
            return;
        }

        Point bottomCenter = target.getBottomMiddlePixel();
        Point worldPos = cameraIntrinsics.calculateWorldPos(bottomCenter.getX(), bottomCenter.getY());

        if (worldPos == null) {
            KLog.d("VisionRoundTrip", () -> String.format("[%s] Failed to convert detection to world coordinates - using fallback", getName()));
            return;
        }

        detectedBallWorldPos = worldPos;
        KLog.d("VisionRoundTrip", () -> String.format("[%s] %s detected (%s): pixel=(%.0f,%.0f) world=(%.1f,%.1f)",
                getName(), target.label, selectionStrategy,
                bottomCenter.getX(), bottomCenter.getY(),
                worldPos.getX(), worldPos.getY()));

        Position currentPos = new Position(SharedData.getOdometryWheelIMUPosition());
        PurePursuitAction moveToBall = getMoveToBall();
        moveToBall.clearPoints();
        moveToBall.addPoint(worldPos.getX(), worldPos.getY(), Math.toDegrees(currentPos.getTheta()));
    }

    private VisionRecognition getTargetRecognition() {
        List<VisionRecognition> all = artifactProcessor.getLatestResult();
        if (all == null || all.isEmpty()) return null;

        List<VisionRecognition> candidates = filterByLabel(all, targetBallColor);
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

    public Point getDetectedBallWorldPos() { return detectedBallWorldPos; }
    public boolean isUsingVision() { return useVision; }
    public boolean hasDetectedBall() { return useVision && detectedBallWorldPos != null; }
}
