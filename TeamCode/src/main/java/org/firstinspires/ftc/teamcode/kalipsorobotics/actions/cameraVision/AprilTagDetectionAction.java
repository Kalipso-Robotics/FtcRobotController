package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.AprilTagConfig.*;

import android.util.Log;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagFieldLayout;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.CameraGimbal;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.FixedCameraMount;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.LimelightAprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.LimelightPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.List;

public class AprilTagDetectionAction extends Action {

    AllianceColor allianceColor;

    private final OpModeUtilities opModeUtilities;

    private final AprilTagCamera camera;
    private final CameraGimbal cameraGimbal;
    private final Turret turret;
    private final int targetAprilTagId;
    private boolean hasStarted = false;
    private final AprilTagFieldLayout fieldLayout;

    private Position camRelAprilTagPos;

    boolean hasFound;
    private double xAprilTagRelToCamMM;
    private double yAprilTagRelToCamMM;
    private double zAprilTagRelToCamMM;
    private double distanceFromCamToAprilTag;

    private double prevPitchDeg = Double.MIN_VALUE;
    private int consecutiveGoodReadings = 0;
    private int consecutiveBadReadings;

    private static final int STABILITY_THRESHOLD = 2;
    public static int CONSECUTIVE_BAD_READING_TOLERANCE = 0;

    private Position prevLimelightGlobalPos;
    private Position globalPos;
    private final double OUT_OF_RANGE_THRESHOLD_MM = 1200;

    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int targetAprilTagId, AllianceColor allianceColor) {
        this(opModeUtilities, turret, targetAprilTagId, allianceColor, new LimelightAprilTagCamera(opModeUtilities, allianceColor));
    }

    /**
     * Allows any AprilTagCamera implementation (e.g. an Arducam-backed one) to be
     * substituted in without touching the detection/filtering logic below. Assumes the
     * camera is rigidly bolted to the turret (this season's rig) - use the overload below
     * once a real pan/tilt CameraMount exists.
     */
    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int targetAprilTagId, AllianceColor allianceColor, AprilTagCamera camera) {
        this(opModeUtilities, turret, targetAprilTagId, allianceColor, camera, new FixedCameraMount(TURRET_REL_CAM_POS));
    }

    /**
     * Allows any CameraGimbal (e.g. a 2-servo pan/tilt CameraMount) to be substituted in
     * without touching the detection/filtering logic below.
     */
    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, int targetAprilTagId, AllianceColor allianceColor, AprilTagCamera camera, CameraGimbal cameraGimbal) {
        this.allianceColor = allianceColor;
        this.opModeUtilities = opModeUtilities;

        fieldLayout = buildFieldLayout();

        this.camera = camera;
        this.camera.start();
        this.cameraGimbal = cameraGimbal;

        this.turret = turret;
        this.targetAprilTagId = targetAprilTagId;
    }

    public AprilTagCamera getCamera() {
        return camera;
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            hasStarted = true;
        }

        List<TagObservation> observations = camera.getLatestObservations();
        hasFound = false;

        for (TagObservation observation : observations) {
            if (observation.getTagId() == targetAprilTagId) {
                Position tagFieldPos = fieldLayout.getFieldPose(observation.getTagId());
                if (tagFieldPos == null) {
                    KLog.d("AprilTag", () -> "No known field position for tag " + observation.getTagId() + ", skipping");
                    continue;
                }

                hasFound = true;

                // ==================== RAW CAMERA DATA ====================
                double rawPitchDeg = observation.getRawPitchDeg();

                // ==================== CALCULATED: Odometry Transform ====================
                camRelAprilTagPos = observation.getCamRelTagPos();

                KLog.d("AprilTag_CAM_REL_APRIL_TAG", () -> String.format("CamRelTag(x=%.1fmm, y=%.1fmm, θ=%.2f°)",
                        camRelAprilTagPos.getX(), camRelAprilTagPos.getY(), Math.toDegrees(camRelAprilTagPos.getTheta())));

                // ==================== CALCULATE GLOBAL POS ====================

                globalPos = calculateGlobalLimelightPosition(tagFieldPos);

                // ==================== SPIKE DETECTION ====================
                boolean isSpike = isLimelightSpike(rawPitchDeg, prevPitchDeg);
                if (isSpike || globalPos == null) {
                    KLog.d("AprilTag_SPIKE", () -> String.format("REJECTED | prev=%.2f° curr=%.2f° delta=%.2f°",
                            prevPitchDeg, rawPitchDeg, rawPitchDeg - prevPitchDeg));
                    SharedData.getLimelightRawPosition().reset();
                    hasFound = false;
                    consecutiveGoodReadings = 0;
                    prevLimelightGlobalPos = new Position(0,0,0);
                    prevPitchDeg = rawPitchDeg;
                    return;
                }

                SharedData.setUnfilteredLimelightGlobalPos(globalPos);

                if (isLimelightOdometrySpike()) {
                    KLog.d("AprilTag_ODOMETRY_SPIKE", () -> String.format("REJECTED | prev=%.2f° curr=%.2f° delta=%.2f°",
                            prevPitchDeg, rawPitchDeg, rawPitchDeg - prevPitchDeg));
                    SharedData.getLimelightRawPosition().reset();
                    hasFound = false;
                    consecutiveGoodReadings = 0;
                    prevLimelightGlobalPos = new Position(0,0,0);
                    prevPitchDeg = rawPitchDeg;
                    return;
                }

                prevPitchDeg = rawPitchDeg;
                prevLimelightGlobalPos = globalPos;
                consecutiveGoodReadings++;
                consecutiveBadReadings = 0;
                consecutiveGoodReadings = Math.min(consecutiveGoodReadings, STABILITY_THRESHOLD + 1);

                // ==================== CALCULATED: Global Position ====================

                if (globalPos != null) {
                    SharedData.setLimelightGlobalPosition(globalPos);
                    KLog.d("AprilTag_GLOBAL", () -> String.format("RobotPos(x=%.1fmm, y=%.1fmm, θ=%.2f°)",
                            globalPos.getX(), globalPos.getY(), Math.toDegrees(globalPos.getTheta())));
                }
                //========================= For Raw Data Stuff To April Tag ======================
                xAprilTagRelToCamMM = observation.getTagRelCamXMM();
                yAprilTagRelToCamMM = observation.getTagRelCamYMM();
                zAprilTagRelToCamMM = observation.getTagRelCamZMM(); // front back offset from tag

                // ==================== CALCULATED: Angle & Distance to Goal ====================
                // Goal offset is fixed relative to AprilTag - always add in positive X direction
                distanceFromCamToAprilTag = Math.hypot(xAprilTagRelToCamMM, zAprilTagRelToCamMM);
                double adjustedX = xAprilTagRelToCamMM;
                double adjustedZ = zAprilTagRelToCamMM + GOAL_OFFSET_REL_APRIL_TAG_IN_CAMERA_SPACE_Z;

                double estimateHeadingFromCamToGoal = Math.atan2(adjustedX, adjustedZ);

                KLog.d("AprilTag_GOAL", () -> String.format("TagPos(x=%.1f, z=%.1f) + Offset -> Adj(x=%.1f, z=%.1f) | AngleToGoal=%.2f° | Dist=%.1fmm",
                        xAprilTagRelToCamMM, zAprilTagRelToCamMM, adjustedX, adjustedZ,
                        Math.toDegrees(estimateHeadingFromCamToGoal), distanceFromCamToAprilTag));

                // ==================== OUTPUT: Send to SharedData ====================
                if (consecutiveGoodReadings > STABILITY_THRESHOLD) {
                    LimelightPos currentRawPos = new LimelightPos(distanceFromCamToAprilTag, estimateHeadingFromCamToGoal, xAprilTagRelToCamMM, yAprilTagRelToCamMM, zAprilTagRelToCamMM);
                    SharedData.setLimelightRawPosition(currentRawPos);
                } else {
                    KLog.d("AprilTag_STABILITY", () -> String.format("Waiting for stable readings: %d/%d", consecutiveGoodReadings, STABILITY_THRESHOLD));
                }
            }
        }

        if (!hasFound) {
            consecutiveGoodReadings = 0;
            consecutiveBadReadings++;
            if (consecutiveBadReadings > CONSECUTIVE_BAD_READING_TOLERANCE) {
                KLog.d("AprilTag", "No AprilTag detected consecutively. Resetting SharedData.");
                SharedData.getLimelightRawPosition().reset();
            }
        }
    }


    /**
     * Calculates the robot's global position based on current vision data.
     * Uses the detected AprilTag position and known tag field location.
     *
     * Coordinate system:
     * - Field: +X is forward from init position, +Y is right, angles are CCW from +X
     * - Camera: +Z is forward (optical axis), +X is right, +Y is down
     * - AprilTag's local frame: +X is the direction the tag faces, origin at tag center
     *
     * @param tagFieldPos Field position of whichever tag was just observed (from fieldLayout).
     * @return Robot's global field position, or null if no valid detection
     */
    public Position calculateGlobalLimelightPosition(Position tagFieldPos) {
        if (!hasFound) {
            return null;
        }

        Position robotRelRobotPos = new Position(0, 0, 0);
        double turretAngle = turret.getCurrentAngleRad();

        Position robotRelTurretPos = robotRelRobotPos.toNewFrame(new Position(ROBOT_REL_TURRET_POINT.getX(), ROBOT_REL_TURRET_POINT.getY(), -turretAngle));
        Position robotRelCamPos = robotRelTurretPos.toNewFrame(cameraGimbal.getMountRelCamPos());
        Position robotRelAprilTagPos = robotRelCamPos.toNewFrame(camRelAprilTagPos);
        Position robotRelFieldPos = robotRelAprilTagPos.toNewFrame(tagFieldPos);
        KLog.d("AprilTag_GLOBAL", () -> "robotRelFieldPos: " + robotRelFieldPos);
        return robotRelFieldPos;
    }

    private boolean isLimelightSpike(double currentPitchDeg, double prevPitchDeg) {
        if (globalPos == null) {
            return false;
        }

        double angleDiff = MathFunctions.angleWrapDeg(currentPitchDeg - prevPitchDeg);

        KLog.d("AprilTagDetectionAction_CheckingSpike", () -> "Current LL pos " + globalPos +
                " prev LL pos " + prevLimelightGlobalPos +
                " Current pitch Deg " + currentPitchDeg +
                " prev pitch Deg " + prevPitchDeg
        );

        if (Math.abs(angleDiff) > 90) {
            KLog.d("AprilTagDetectionAction_Spike", () -> "Delta angle too high, delta: " + angleDiff + " deg");
            return true;
        }
        if (Math.abs(globalPos.getX()) > 3600 || Math.abs(globalPos.getY()) > 3000) {
            KLog.d("AprilTagDetectionAction_Spike", () -> "Pos out of the field, pos: " + globalPos);
            return true;
        }

        if (prevLimelightGlobalPos != null && !prevLimelightGlobalPos.isEmpty()) {
            double dist = globalPos.distanceTo(prevLimelightGlobalPos);

            if (!prevLimelightGlobalPos.isEmpty() && dist > OUT_OF_RANGE_THRESHOLD_MM) {
                KLog.d("AprilTagDetectionAction_Spike", () -> "Distance to previous position too high, distance: " + dist + " mm" + " previous pos: " + prevLimelightGlobalPos + " curr: " + globalPos);
                return true;
            }
        }
        return false;
    }

    private boolean isLimelightOdometrySpike() {

        Position odoPos = SharedData.getOdometryWheelIMUPosition();

        if (globalPos.distanceTo(odoPos) > 600) {
            KLog.d("AprilTagDetectionAction_Spike", () -> "Delta rel to Odometry too high, LL pos: " + globalPos + " Odometry " + odoPos);
            return true;
        }

        return false;

    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}

