package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
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
import java.util.Locale;

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
                    KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                            "[%s] frame=%d dist-to-lookout=%.0fmm pos=(%.0f,%.0f) cam=%s",
                            getName(), fc, d,
                            currentPos.toPoint().getX(), currentPos.toPoint().getY(),
                            artifactProcessor.getDiagnosticSummary()));
                }
                if (dist < lookoutRadiusMM) {
                    // Single attempt at the lookout. Whether vision finds a ball
                    // or not, we move on — the pre-configured path (lookout →
                    // launch) keeps executing if vision returns nothing.
                    final double d = dist;
                    KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                            "[%s] At lookout (%.0fmm) - processing vision", getName(), d));
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
        PurePursuitAction moveToBall = getMoveToBall();
        int polarity = SharedData.getAllianceColor().getPolarity();
        double launchHeading = 90 * polarity;
        Position robotPos = new Position(SharedData.getOdometryWheelIMUPosition());

        KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                        "[%s] processVision START robot=(%.1f,%.1f,%.1f°) launchPt=(%.1f,%.1f) heading=%.1f° polarity=%d",
                        getName(), robotPos.getX(), robotPos.getY(), Math.toDegrees(robotPos.getTheta()),
                        launchPoint.getX(), launchPoint.getY(), launchHeading, polarity));

        VisionRecognition target = getTargetRecognition();
        if (target == null) {
        KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                            "[%s] NO BALL DETECTED - keeping fallback path. cam=%s",
                            getName(), artifactProcessor.getDiagnosticSummary()));
        return;
        }

        Point bottomCenter = target.getBottomMiddlePixel();
        Point worldPos = cameraIntrinsics.calculateWorldPos(bottomCenter.getX(), bottomCenter.getY(), robotPos);
        if (worldPos == null) {
        KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                            "[%s] WORLD CONVERSION FAILED for pixel=(%.1f,%.1f) - keeping fallback path",
                            getName(), bottomCenter.getX(), bottomCenter.getY()));
        return;
        }

        detectedBallWorldPos = worldPos;

        Point sectorGrabPoint = chooseSectorGrabPoint(worldPos, polarity);
        KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                        "[%s] SELECTED %s conf=%.2f pixel=(%.1f,%.1f) ballWorld=(%.1f,%.1f) "
                        + "→ sectorGrab=(%.1f,%.1f) → launch=(%.1f,%.1f) heading=%.1f°",
                getName(), target.label, target.confidence,
                bottomCenter.getX(), bottomCenter.getY(),
                worldPos.getX(), worldPos.getY(),
                sectorGrabPoint.getX(), sectorGrabPoint.getY(),
                launchPoint.getX(), launchPoint.getY(), launchHeading));

        moveToBall.clearPoints();
        moveToBall.addPoint(sectorGrabPoint.getX(), sectorGrabPoint.getY(), Math.toDegrees(robotPos.getTheta()));
        moveToBall.addPoint(launchPoint.getX(), launchPoint.getY(), launchHeading);
        moveToBall.rebuildPath();
        logPlannedPath(moveToBall);

        // Original "drive straight to ball" path — replaced by sector grab logic above.
        // moveToBall.addPoint(worldPos.getX(), worldPos.getY(), Math.toDegrees(robotPos.getTheta()));
        // moveToBall.addPoint(launchPoint.getX(), launchPoint.getY(), launchHeading);
    }

    private static final double[] SECTOR_CENTERS_X = {450, 1350, 2250, 3150};
    private static final double SECTOR_GRAB_Y_ABS = 1050;

    private Point chooseSectorGrabPoint(Point ballWorld, int polarity) {
        double bestDx = Double.MAX_VALUE;
        double bestX = SECTOR_CENTERS_X[0];
        int bestIdx = 0;
        StringBuilder sb = new StringBuilder();
        sb.append('[').append(getName()).append("] sector eval ballX=")
                .append(String.format(Locale.US, "%.1f", ballWorld.getX())).append(" polarity=")
                .append(polarity).append(" candidates:");
        for (int i = 0; i < SECTOR_CENTERS_X.length; i++) {
            double sx = SECTOR_CENTERS_X[i];
            double d = Math.abs(ballWorld.getX() - sx);
            sb.append(String.format(Locale.US, " S%d(x=%.0f,dx=%.1f)", i, sx, d));
            if (d < bestDx) {
                bestDx = d;
                bestX = sx;
                bestIdx = i;
            }
        }
        double grabY = SECTOR_GRAB_Y_ABS * polarity;
        final String evalMsg = sb.toString();
        final int chosenIdx = bestIdx;
        final double chosenX = bestX;
        final double chosenDx = bestDx;
        KLog.d("VisionRoundTrip", () -> evalMsg);
        KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                "[%s] sector CHOSEN S%d centerX=%.1f dx=%.1f grabY=%.1f → grab=(%.1f,%.1f)",
                getName(), chosenIdx, chosenX, chosenDx, grabY, chosenX, grabY));
        return new Point(bestX, grabY);
    }


    private void logPlannedPath(PurePursuitAction moveToBall) {
        List<Position> path = moveToBall.getPathPoints();
        StringBuilder sb = new StringBuilder();
        sb.append('[').append(getName()).append("] planned path (").append(path.size()).append(" pts):");
        for (int i = 0; i < path.size(); i++) {
            Position p = path.get(i);
            sb.append(String.format(Locale.US, " #%d=(%.1f,%.1f,%.1f°)",
                    i, p.getX(), p.getY(), Math.toDegrees(p.getTheta())));
        }
        final String msg = sb.toString();
        KLog.d("VisionRoundTrip", () -> msg);
    }

    private VisionRecognition getTargetRecognition() {
        List<VisionRecognition> all = artifactProcessor.getLatestResult();
        KLog.d("VisionRoundTrip", () -> String.format(Locale.US, "[%s] Vision: %s | %s",
                getName(),
                all == null ? "NULL" : "count=" + all.size(),
                artifactProcessor.getDiagnosticSummary()));
        if (all == null || all.isEmpty()) return null;

        List<VisionRecognition> candidates = filterByLabel(all, targetBallColor);
        KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                "[%s] After label filter (%s): %d candidates",
                getName(), targetBallColor == null ? "ANY" : targetBallColor, candidates.size()));
        logCandidatesWithWorld(candidates);
        if (candidates.isEmpty()) return null;

        VisionRecognition chosen;
        switch (selectionStrategy) {
            case LARGEST_AREA:
                chosen = BlobUtils.findLargestByArea(candidates);
                break;
            case CLOSEST_TO_CAMERA_CENTER:
                chosen = BlobUtils.findClosestToCameraCenter(candidates, cameraIntrinsics.getCx(), cameraIntrinsics.getCy());
                break;
            case CLOSEST_TO_ROBOT_WORLD:
                Position robotPos = new Position(SharedData.getOdometryWheelIMUPosition());
                chosen = BlobUtils.findClosestToRobotWorld(candidates, cameraIntrinsics, robotPos);
                break;
            case MOST_CIRCULAR:
                chosen = BlobUtils.findMostCircular(candidates);
                break;
            default:
                chosen = candidates.get(0);
        }
        final VisionRecognition c = chosen;
        KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                "[%s] Strategy=%s chose %s conf=%.2f area=%.0f pixel=(%.1f,%.1f)",
                getName(), selectionStrategy,
                c == null ? "NULL" : c.label,
                c == null ? 0f : c.confidence,
                c == null ? 0.0 : c.getArea(),
                c == null ? 0.0 : c.getBottomMiddlePixel().getX(),
                c == null ? 0.0 : c.getBottomMiddlePixel().getY()));
        return chosen;
    }

    private void logCandidatesWithWorld(List<VisionRecognition> candidates) {
        Position robotPos = new Position(SharedData.getOdometryWheelIMUPosition());
        for (int i = 0; i < candidates.size(); i++) {
            VisionRecognition r = candidates.get(i);
            Point px = r.getBottomMiddlePixel();
            Point world = cameraIntrinsics.calculateWorldPos(px.getX(), px.getY(), robotPos);
            final int idx = i;
            final String worldStr = world == null
                    ? "world=NULL"
                    : String.format(Locale.US, "world=(%.1f,%.1f)", world.getX(), world.getY());
            KLog.d("VisionRoundTrip", () -> String.format(Locale.US,
                    "[%s]   cand#%d %s conf=%.2f area=%.0f pixel=(%.1f,%.1f) %s",
                    getName(), idx, r.label, r.confidence, r.getArea(),
                    px.getX(), px.getY(), worldStr));
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
