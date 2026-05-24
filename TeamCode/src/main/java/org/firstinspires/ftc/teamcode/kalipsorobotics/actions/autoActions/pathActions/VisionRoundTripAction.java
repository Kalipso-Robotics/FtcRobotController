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
    private final Point visionLookoutPoint;
    private final double lookoutRadiusMM;
    private boolean visionProcessed = false;
    private Point detectedBallWorldPos;
    private int frameCount = 0;

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
            throw new IllegalArgumentException("Vision mode requires artifactProcessor and cameraIntrinsics");
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

        public Builder(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                       TurretAutoAlign turretAutoAlign, Shooter shooter,
                       Stopper stopper, Intake intake) {
            this.opModeUtilities = opModeUtilities;
            this.driveTrain = driveTrain;
            this.turretAutoAlign = turretAutoAlign;
            this.shooter = shooter;
            this.stopper = stopper;
            this.intake = intake;
        }

        public Builder setTargetPoint(Point targetPoint) { this.targetPoint = targetPoint; return this; }
        public Builder setLaunchPoint(Point launchPoint) { this.launchPoint = launchPoint; return this; }
        public Builder setWaitForShooterReadyMS(double waitMS) { this.waitForShooterReadyMS = waitMS; return this; }
        public Builder setShouldRunIntake(boolean v) { this.shouldRunIntake = v; return this; }
        public Builder setShouldDependOnFlywheel(boolean v) { this.shouldDependOnFlywheel = v; return this; }

        public Builder setVisionLookoutPoint(Point lookoutPoint, double radiusMM) {
            this.visionLookoutPoint = lookoutPoint;
            this.lookoutRadiusMM = radiusMM;
            return this;
        }

        public Builder setVisionLookoutPoint(Point lookoutPoint) {
            return setVisionLookoutPoint(lookoutPoint, 250);
        }

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
                                    CameraIntrinsics cameraIntrinsics, String targetBallColor) {
            return enableVision(artifactProcessor, cameraIntrinsics, targetBallColor, BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
        }

        public Builder enableVision(KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                                    CameraIntrinsics cameraIntrinsics) {
            return enableVision(artifactProcessor, cameraIntrinsics, null, BlobSelectionStrategy.CLOSEST_TO_CAMERA_CENTER);
        }

        public Builder enableVision(KVisionProcessor<List<VisionRecognition>> artifactProcessor,
                                    CameraIntrinsics cameraIntrinsics, BlobSelectionStrategy selectionStrategy) {
            return enableVision(artifactProcessor, cameraIntrinsics, null, selectionStrategy);
        }

        public VisionRoundTripAction build() { return new VisionRoundTripAction(this); }
    }

    @Override
    protected void beforeUpdate() {
        frameCount++;

        if (useVision && !visionProcessed) {
            if (visionLookoutPoint != null) {
                Position currentPos = SharedData.getOdometryWheelIMUPosition();
                double dist = currentPos.toPoint().distanceTo(visionLookoutPoint);
                if (frameCount % 30 == 0) {
                    final int fc = frameCount;
                    final double d = dist;
                    KLog.d("VisionRoundTrip", () -> String.format(
                            "[%s] frame=%d dist-to-lookout=%.0fmm pos=(%.0f,%.0f) cam=%s",
                            getName(), fc, d,
                            currentPos.toPoint().getX(), currentPos.toPoint().getY(),
                            artifactProcessor.getDiagnosticSummary()));
                }
                if (dist < lookoutRadiusMM) {
                    final double d = dist;
                    KLog.d("VisionRoundTrip", () -> String.format("[%s] At lookout (%.0fmm) - processing vision", getName(), d));
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
        PurePursuitAction moveToBall = getMoveToBall();

        if (target == null) {
            KLog.d("VisionRoundTrip", () -> String.format("[%s] No ball - returning to launch (%.0f,%.0f)",
                    getName(), launchPoint.getX(), launchPoint.getY()));
            moveToBall.addPoint(launchPoint.getX(), launchPoint.getY(), 0);
            return;
        }

        Point bottomCenter = target.getBottomMiddlePixel();
        Point worldPos = cameraIntrinsics.calculateWorldPos(bottomCenter.getX(), bottomCenter.getY());
        if (worldPos == null) {
            KLog.d("VisionRoundTrip", () -> String.format("[%s] World conversion failed - returning to launch (%.0f,%.0f)",
                    getName(), launchPoint.getX(), launchPoint.getY()));
            moveToBall.addPoint(launchPoint.getX(), launchPoint.getY(), 0);
            return;
        }

        detectedBallWorldPos = worldPos;
        KLog.d("VisionRoundTrip", () -> String.format("[%s] Ball %s: pixel=(%.0f,%.0f) world=(%.1f,%.1f) → launch=(%.0f,%.0f)",
                getName(), target.label,
                bottomCenter.getX(), bottomCenter.getY(),
                worldPos.getX(), worldPos.getY(),
                launchPoint.getX(), launchPoint.getY()));

        Position currentPos = new Position(SharedData.getOdometryWheelIMUPosition());
        moveToBall.clearPoints();
        moveToBall.addPoint(worldPos.getX(), worldPos.getY(), Math.toDegrees(currentPos.getTheta()));
        moveToBall.addPoint(launchPoint.getX(), launchPoint.getY(), 0);
    }

    private VisionRecognition getTargetRecognition() {
        List<VisionRecognition> all = artifactProcessor.getLatestResult();
        KLog.d("VisionRoundTrip", () -> String.format("[%s] Vision: %s | %s",
                getName(),
                all == null ? "NULL" : "count=" + all.size(),
                artifactProcessor.getDiagnosticSummary()));
        if (all == null || all.isEmpty()) return null;

        List<VisionRecognition> candidates = filterByLabel(all, targetBallColor);
        if (candidates.isEmpty()) return null;

        switch (selectionStrategy) {
            case LARGEST_AREA:
                return BlobUtils.findLargestByArea(candidates);
            case CLOSEST_TO_CAMERA_CENTER:
                return BlobUtils.findClosestToCameraCenter(candidates, cameraIntrinsics.getCx(), cameraIntrinsics.getCy());
            case CLOSEST_TO_ROBOT_WORLD:
                Position robotPos = new Position(SharedData.getOdometryWheelIMUPosition());
                return BlobUtils.findClosestToRobotWorld(candidates, cameraIntrinsics, robotPos.toPoint());
            case MOST_CIRCULAR:
                return BlobUtils.findMostCircular(candidates);
            default:
                return candidates.get(0);
        }
    }

    private List<VisionRecognition> filterByLabel(List<VisionRecognition> recognitions, String label) {
        if (label == null) return recognitions;
        List<VisionRecognition> filtered = new ArrayList<>();
        for (VisionRecognition r : recognitions) {
            if (label.equals(r.label)) filtered.add(r);
        }
        return filtered;
    }

    public Point getDetectedBallWorldPos() { return detectedBallWorldPos; }
    public boolean isUsingVision() { return useVision; }
    public boolean hasDetectedBall() { return useVision && detectedBallWorldPos != null; }
}
