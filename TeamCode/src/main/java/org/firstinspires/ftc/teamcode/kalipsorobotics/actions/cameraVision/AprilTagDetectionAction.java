package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.LimelightAprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;

import java.util.List;
import java.util.Locale;

/**
 * Turns HIVE tag sightings into (1) a robot field position for re-zeroing odometry and (2) the goal state (which of
 * our CELLs is up), both on SharedData. The math is HiveTagLocalizer; this class only feeds it and publishes.
 * Runs on its own thread (OpModeUtilities.runAprilTagExecutorService).
 *
 * The camera sits behind AprilTagCamera, so the Arducam (KAprilTagProcessor) can replace the Limelight:
 *   new AprilTagDetectionAction(opModeUtilities, turret, alliance, kAprilTagProcessor);
 *
 * Heading comes from odometry (IMU), so a fix only moves x and y.
 * ponytail: uses the pose at processing time, not at frame capture; error ~ speed * Limelight latency (~40 ms).
 * Re-zero while slow, or look the pose up with SharedData.getOdometryWheelIMUPositionAt if that is ever not enough.
 */
public class AprilTagDetectionAction extends Action {
    public static final String TAG = "Vision_Limelight";

    private final OpModeUtilities opModeUtilities;
    private final AprilTagCamera camera;
    private final Turret turret;
    private final AllianceColor allianceColor;

    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor) {
        this(opModeUtilities, turret, allianceColor, new LimelightAprilTagCamera(opModeUtilities, allianceColor));
    }

    public AprilTagDetectionAction(OpModeUtilities opModeUtilities, Turret turret, AllianceColor allianceColor, AprilTagCamera camera) {
        this.opModeUtilities = opModeUtilities;
        this.turret = turret;
        this.allianceColor = allianceColor;
        this.camera = camera;
        this.camera.start();
    }

    public AprilTagCamera getCamera() {
        return camera;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    @Override
    protected void update() {
        List<TagObservation> observations = camera.getLatestObservations();
        if (observations.isEmpty()) return;

        double heading = SharedData.getOdometryWheelIMUPosition().getTheta();
        HiveTagLocalizer.Fix fix = HiveTagLocalizer.solve(observations, heading, turret.getAngleRad(), allianceColor);

        if (fix.upCell != null) {
            HiveTagLocalizer.Cell previous = SharedData.getHiveUpCell();
            SharedData.setHiveUpCell(fix.upCell);
            if (previous != fix.upCell) {
                KLog.d(TAG, () -> "event=goalState upCell=" + fix.upCell + " was=" + previous);
            }
        }

        // No turret encoder: mid-swing, the commanded angle is not where the camera points.
        if (fix.robot == null || turret.isMoving()) return;
        SharedData.setLimelightGlobalPosition(new Position(fix.robot.getX(), fix.robot.getY(), heading));
        if (KLog.isDebug()) {
            KLog.d(TAG, () -> String.format(Locale.US, "event=fix x=%.0f y=%.0f tags=%d", fix.robot.getX(), fix.robot.getY(), fix.tagsUsed));
        }
    }
}
