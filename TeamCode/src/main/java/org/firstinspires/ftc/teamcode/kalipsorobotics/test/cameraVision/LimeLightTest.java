package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.AprilTagDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToLimelight;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.LimelightAprilTagCamera;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.TagObservation;

import java.util.List;
import java.util.Locale;

/**
 * Limelight check through the real code path (LimelightAprilTagCamera -> AprilTagDetectionAction -> SharedData).
 * Shows the tag IDs and ranges seen, the age of the last position fix and the Hive state (which Cell is up).
 * GP1 A: re-zero odometry to the Limelight fix (ResetOdometryToLimelight) and show the pose before and after.
 * The camera starts once, in init. Odometry starts at (0,0,0).
 * NOTE: until the real HiveTagLocalizer is pushed (placeholder now), there is no fix and no Hive state.
 */
@TeleOp(name = "LimeLightTest", group = "Test")
public class LimeLightTest extends KOpMode {
    public static final String TAG = "Vision_Limelight";

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Turret.setInstanceNull();
        Turret turret = Turret.getInstance(opModeUtilities);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, new Position(0, 0, 0));

        LimelightAprilTagCamera camera = new LimelightAprilTagCamera(opModeUtilities, allianceColor);
        AprilTagDetectionAction detection = new AprilTagDetectionAction(opModeUtilities, turret, allianceColor, camera); // starts the camera once
        detection.setName("limelightTest");
        KLog.d(TAG, () -> "event=init alliance=" + allianceColor);

        String lastReset = "none yet";
        waitForStart();
        while (opModeIsActive()) {
            detection.updateCheckDone();
            List<TagObservation> observations = camera.getLatestObservations();

            if (kGamePad1.isButtonAFirstPressed()) {
                Position before = SharedData.getOdometryWheelIMUPosition();
                ResetOdometryToLimelight reset = new ResetOdometryToLimelight(turret);
                reset.updateCheckDone();
                Position after = SharedData.getOdometryWheelIMUPosition();
                lastReset = "before " + fmt(before) + " -> after " + fmt(after);
                String logged = lastReset;
                KLog.d(TAG, () -> "event=reset " + logged);
            }

            if (org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer.IS_PLACEHOLDER) {
                telemetry.addLine("WARNING: HiveTagLocalizer is a placeholder -- no fix, no Hive state, A does nothing");
            }
            telemetry.addLine("GP1 A = re-zero odometry to the Limelight fix");
            telemetry.addData("tags seen", observations.size());
            for (TagObservation o : observations) {
                telemetry.addData("  tag " + o.getTagId(), "range %.0f mm", o.getGroundRangeMM());
            }
            double ageMs = SharedData.getLimelightFixAgeMs();
            telemetry.addData("fix age", "%.0f ms", ageMs);
            telemetry.addData("fix", fmt(SharedData.getLimelightGlobalPosition()));
            telemetry.addData("Hive up cell", SharedData.getHiveUpCell() == null ? "not seen" : SharedData.getHiveUpCell());
            telemetry.addData("odometry", fmt(SharedData.getOdometryWheelIMUPosition()));
            telemetry.addData("last reset", lastReset);
            telemetry.update();
            if (KLog.isDebug()) {
                KLog.d(TAG, () -> "tags=" + observations.size() + " fixAgeMs=" + ageMs + " upCell=" + SharedData.getHiveUpCell());
            }
        }
        camera.stop();
        cleanupRobot();
    }

    private static String fmt(Position p) {
        if (p == null) {
            return "none";
        }
        return String.format(Locale.US, "x=%.0f y=%.0f h=%.1f deg", p.getX(), p.getY(), Math.toDegrees(p.getTheta()));
    }
}
