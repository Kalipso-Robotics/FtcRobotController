package org.firstinspires.ftc.teamcode.kalipsorobotics.test.cameraVision;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import java.util.Locale;

/**
 * THE test. Drive the robot wherever you like, bring it back to where it started, and see
 * whether the localizer agrees that you did.
 *
 * Run it after OctoStartup, OctoTune, AND OctoTune's own confirmation pass (its renderDone()
 * tells you to run it once more after pasting the constants back in -- that is the only check
 * that the numbers survived the trip through source, and it is not optional). This OpMode is
 * the step after that, not a replacement for it: it exercises the thing you actually care
 * about, the pose the pathing code will read, through the same boundary function, over an
 * arbitrary driven path rather than a scripted one.
 *
 * This is the only OpMode of the three that drives motors, because driving it is the point.
 *
 * WHY THE EXTRA NUMBERS
 *
 * "It came back within 20 mm" is not a result on its own. Odometry error accumulates, so the
 * same 20 mm is excellent after a long aggressive drive and awful after nudging the robot in a
 * small circle. But percent-of-distance alone is also the wrong single answer: a two-pod plus
 * IMU localizer's error is dominated by HEADING error, and position error goes roughly as the
 * integral of heading error along the path. That grows with distance, with total angle turned,
 * and with elapsed time through IMU bias. Normalising only by distance flatters a robot driven
 * straight down a field and damns one pirouetting in a two metre box, and auto is the
 * pirouetting case.
 *
 * So this reports closure against all of them: distance travelled, total angle turned, and
 * elapsed time. Judge the closure against the drive that produced it.
 *
 * The health row is not decoration either. Without crcOk, the localizer status and the loop
 * period, a dropped I2C frame looks exactly like a robot that drifted.
 *
 *   adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs ~/
 */
@TeleOp
public class OctoTest extends LinearOpMode {

    private final OctoQuad.LocalizerDataBlock loc = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock   enc = new OctoQuad.EncoderDataBlock();
    private final OctoConfig.Pose pose = new OctoConfig.Pose();
    private final ElapsedTime runtime = new ElapsedTime();

    private OctoQuad q;
    private KFileWriter csv;

    // Odometer state: how hard was this test, not just how wrong was the answer.
    private double pathMm, turnedDeg;
    private double prevX, prevY;
    private boolean prevSeeded;

    // Unwrapped heading, for total-angle-turned. Same reasoning as OctoTune.updateHeadingUnwrap.
    private double lastRawHeadingRad;
    private boolean headingSeeded;

    private int rawX0, rawY0, rawX20;
    private int badReads, stalls;
    private int legacyBack0;

    @Override
    public void runOpMode() {

        q = hardwareMap.get(OctoQuad.class, OctoConfig.HARDWARE_NAME);
        OctoConfig.apply(q);

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);

        csv = new KFileWriter("OctoTest", opModeUtilities);
        csv.writeLine(String.format(Locale.US,
                "# constants cpmX=%.5f cpmY=%.5f tcpX=%.2f tcpY=%.2f scale=%.6f mirror=%b",
                OctoConfig.COUNTS_PER_MM_X, OctoConfig.COUNTS_PER_MM_Y,
                OctoConfig.TCP_OFFSET_MM_X, OctoConfig.TCP_OFFSET_MM_Y,
                OctoConfig.IMU_HEADING_SCALAR, OctoConfig.MIRROR_BOARD_FRAME));
        csv.writeLine("t_s,x_mm,y_mm,heading_deg,closure_mm,path_mm,turned_deg,"
                + "pod_deg,disagree_deg,rawX,rawY,rawX2,legacy_back,crcOk,status,loop_ms,stall");

        telemetry.addLine("OCTO TEST -- drive it around, bring it home.");
        telemetry.addLine();
        telemetry.addLine("Left stick drives and strafes, right stick turns.");
        telemetry.addLine("B re-zeros the origin wherever the robot is now.");
        telemetry.addLine();
        telemetry.addLine("Mark the robot's starting spot on the floor first, so you can");
        telemetry.addLine("put it back accurately. The test is only as good as that mark.");
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) { csv.close(); return; }

        if (!OctoConfig.calibrateImu(q, this)) {
            telemetry.addLine("IMU calibration failed. Not a usable test.");
            telemetry.update();
            csv.writeLine("# ABORT imu calibration failed");
            csv.close();
            sleep(4000);
            return;
        }

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        runtime.reset();
        // One read before zeroing, so the software count offsets are taken against real
        // hardware values. OctoConfig.apply() no longer calls resetEverything(), so the board
        // still holds whatever counts the previous OpMode left on it.
        q.readLocalizerDataAndAllEncoderData(loc, enc);
        zero(driveTrain);

        ElapsedTime loopTimer = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                double loopMs = loopTimer.milliseconds();
                loopTimer.reset();
                boolean stalled = loopMs > OctoConfig.LOOP_STALL_MS;
                if (stalled) stalls++;

                q.readLocalizerDataAndAllEncoderData(loc, enc);
                boolean dataOk = loc.isDataValid() && enc.isDataValid();

                if (dataOk) {
                    OctoConfig.toRobotFrame(loc, pose);
                    updateOdometer(loc.heading_rad);
                } else {
                    badReads++;
                }

                driveAction.move(gamepad1);
                if (gamepad1.bWasPressed()) zero(driveTrain);

                double closure = Math.hypot(pose.x, pose.y);
                double headErr = wrapDeg(pose.headingDeg);
                double podDeg = podHeading();
                int legacyBack = driveTrain.getBackEncoder().getCurrentPosition() - legacyBack0;

                csv.writeLine(String.format(Locale.US,
                        "%.3f,%.2f,%.2f,%.4f,%.2f,%.1f,%.2f,%.4f,%.4f,%d,%d,%d,%d,%b,%s,%.2f,%b",
                        runtime.seconds(), pose.x, pose.y, pose.headingDeg, closure,
                        pathMm, turnedDeg, podDeg, pose.headingDeg - podDeg,
                        rawX(), rawY(), rawX2(), legacyBack,
                        loc.crcOk, loc.localizerStatus, loopMs, stalled));

                render(closure, headErr, podDeg, legacyBack, loopMs, badReads, stalls);
                telemetry.update();
            }
        } finally {
            csv.close();
        }
    }

    // ---------------------------------------------------------------------------- render

    private void render(double closure, double headErr, double podDeg,
                        int legacyBack, double loopMs, int bad, int stall) {

        boolean posOk = closure <= OctoConfig.TOL_TEST_CLOSURE_MM;
        boolean hdgOk = Math.abs(headErr) <= OctoConfig.TOL_TEST_HEADING_DEG;

        telemetry.addLine(posOk && hdgOk ? "=== WITHIN TOLERANCE ===" : "=== OUT OF TOLERANCE ===");
        telemetry.addData("closure", "%8.1f mm   limit %5.1f   %s",
                closure, OctoConfig.TOL_TEST_CLOSURE_MM, posOk ? "PASS" : "FAIL");
        telemetry.addData("heading", "%8.2f deg  limit %5.1f   %s",
                headErr, OctoConfig.TOL_TEST_HEADING_DEG, hdgOk ? "PASS" : "FAIL");
        telemetry.addLine();
        telemetry.addData("x / y", "%8.1f / %8.1f mm", pose.x, pose.y);
        telemetry.addLine();

        // How hard was this test. Closure means nothing without these.
        telemetry.addLine("--- how far you drove ---");
        telemetry.addData("path",    "%8.1f m", pathMm / 1000.0);
        telemetry.addData("turned",  "%8.0f deg total", turnedDeg);
        telemetry.addData("elapsed", "%8.0f s", runtime.seconds());
        if (pathMm > 500) {
            telemetry.addData("drift", "%8.2f %% of path", closure / pathMm * 100.0);
        }
        telemetry.addLine();

        telemetry.addLine("--- health (a dropped frame looks like drift) ---");
        telemetry.addData("imu vs pods", "%8.2f deg  (limit %.1f)",
                pose.headingDeg - podDeg, OctoConfig.TOL_DISAGREE_DEG);
        telemetry.addData("status", "%s  crc %b", loc.localizerStatus, loc.crcOk);
        telemetry.addData("loop",   "%8.1f ms   bad reads %d   stalls %d", loopMs, bad, stall);
        telemetry.addLine();

        // Sign-only cross-check against the legacy odometry, which is the thing that decides
        // whether swapping this localizer into the pathing code mirrors every path. The legacy
        // +Y direction is not determined by reading the source: it is set by the back encoder's
        // wiring sign, which no file records. So compare signs here, with the real hardware.
        telemetry.addLine("--- legacy odometry cross-check (signs only) ---");
        telemetry.addData("octo y", "%8.1f mm", pose.y);
        telemetry.addData("legacy back enc", "%8d ticks", legacyBack);
        telemetry.addLine("Strafe RIGHT and confirm both agree on which way is positive.");
        telemetry.addLine("If they disagree, localization/Odometry and OctoConfig do not");
        telemetry.addLine("share a +Y and wiring this in will mirror every path.");
        telemetry.addLine();
        telemetry.addLine("B re-zeros the origin here.");
    }

    // --------------------------------------------------------------------------- helpers

    /**
     * Path length and total angle turned, accumulated per loop.
     *
     * Heading is unwrapped via OctoConfig.wrapDeltaRad, the same way OctoTune does it: whether
     * the board's heading_rad accumulates or wraps at +/-pi is undocumented, and summing bounded
     * deltas is correct either way.
     */
    private void updateOdometer(double rawHeadingRad) {
        if (prevSeeded) {
            pathMm += Math.hypot(pose.x - prevX, pose.y - prevY);
        }
        prevX = pose.x;
        prevY = pose.y;
        prevSeeded = true;

        if (!headingSeeded) {
            lastRawHeadingRad = rawHeadingRad;
            headingSeeded = true;
            return;
        }
        double d = OctoConfig.wrapDeltaRad(rawHeadingRad, lastRawHeadingRad);
        lastRawHeadingRad = rawHeadingRad;
        turnedDeg += Math.abs(Math.toDegrees(d));
    }

    /**
     * Closure heading normalised to +/-180.
     *
     * "Did it end up facing the same way" is a question about orientation, not about how many
     * times it spun on the way. Five full turns and back should read zero error, not 1800.
     */
    private static double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d >  180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }

    private double podHeading() {
        return OctoConfig.monitorHeadingDeg(rawX(), rawX2());
    }

    private int rawX()  { return enc.positions[OctoConfig.CH_X]  - rawX0;  }
    private int rawY()  { return enc.positions[OctoConfig.CH_Y]  - rawY0;  }
    private int rawX2() { return enc.positions[OctoConfig.CH_X2] - rawX20; }

    /** Software zeroing, for the same reason OctoTune does it: see OctoTune.zero(). */
    private void zero(DriveTrain driveTrain) {
        q.setLocalizerPose(0, 0, 0f);
        rawX0  = enc.positions[OctoConfig.CH_X];
        rawY0  = enc.positions[OctoConfig.CH_Y];
        rawX20 = enc.positions[OctoConfig.CH_X2];
        legacyBack0 = driveTrain.getBackEncoder().getCurrentPosition();
        pose.x = pose.y = pose.headingDeg = 0;
        pathMm = turnedDeg = 0;
        prevSeeded = false;
        headingSeeded = false;
        csv.writeLine("# --- zeroed at " + String.format(Locale.US, "%.1f s", runtime.seconds()));
        try {
            csv.flush();
        } catch (java.io.IOException e) {
            telemetry.addLine("CSV flush failed: " + e.getMessage());
        }
    }
}
