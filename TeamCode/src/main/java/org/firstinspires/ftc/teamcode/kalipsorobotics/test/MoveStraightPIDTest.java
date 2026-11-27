package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.MoveStraightPIDAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Repeatable MoveStraightPIDAction runs for tuning. Pattern source: IntakeRatioShotTest / RPSPredictionDataCollector
 * (KOpMode + startOdometry, field-centric drive between runs, button-started action, KLog key=value results).
 *
 * Each run is a new MoveStraightPIDAction, so Dashboard edits to its gains (MoveStraightPIDAction: P_XY, D_XY, S_XY,
 * P_ANGLE, S_ANGLE, tolerances, SETTLE_LOOPS) and to DISTANCE_MM / MAX_POWER here apply on the next run.
 *
 * Results per run (telemetry + KLog tag Test_MoveStraightPID, event=result):
 *   along   = how far past (+) or short (-) of the target, along the start heading
 *   cross   = sideways miss (+ = left of the line, field-frame CCW)
 *   heading = final heading minus start heading
 *   peakOvershoot = furthest it went past the target during the run
 * Tape-measure the real travel too: if odometry says on target but the tape doesn't, it's odometry, not the PID.
 *
 * Controls (GP1):
 *   sticks             field-centric drive to set up (ignored while a run is going)
 *   A / B              run forward / backward DISTANCE_MM
 *   dpad up / down     DISTANCE_MM +/- 50
 *   dpad right / left  MAX_POWER +/- 0.1
 *   X                  cancel the run
 */
@Config
@TeleOp
public class MoveStraightPIDTest extends KOpMode {
    public static final String TAG = "Test_MoveStraightPID";
    public static double DISTANCE_MM = 600;
    public static double MAX_POWER = MoveStraightPIDAction.DEFAULT_MAX_POWER;
    public static double TOLERANCE_MM = MoveStraightPIDAction.DEFAULT_TOLERANCE_MM;
    public static double TIMEOUT_MS = MoveStraightPIDAction.DEFAULT_TIMEOUT_MS;
    private static final double DISTANCE_STEP_MM = 50;
    private static final double POWER_STEP = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        FieldOrientedDriveAction driveAction = new FieldOrientedDriveAction(driveTrain, imuModule);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null);

        MoveStraightPIDAction move = null;
        Position start = null;
        double runDistance = 0;
        double peakOvershoot = 0;
        ElapsedTime runTimer = new ElapsedTime();
        String lastResult = "none yet";
        int runCount = 0;

        telemetry.addLine("A/B: run fwd/back   dpad up/down: distance   dpad right/left: max power   X: cancel");
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            boolean running = move != null && !move.getIsDone();

            if (kGamePad1.isDpadUpFirstPressed()) {
                DISTANCE_MM += DISTANCE_STEP_MM;
            }
            if (kGamePad1.isDpadDownFirstPressed()) {
                DISTANCE_MM = Math.max(DISTANCE_STEP_MM, DISTANCE_MM - DISTANCE_STEP_MM);
            }
            if (kGamePad1.isDpadRightFirstPressed()) {
                MAX_POWER = MathFunctions.clamp(MAX_POWER + POWER_STEP, 0.1, 1.0);
            }
            if (kGamePad1.isDpadLeftFirstPressed()) {
                MAX_POWER = MathFunctions.clamp(MAX_POWER - POWER_STEP, 0.1, 1.0);
            }

            boolean forward = kGamePad1.isButtonAFirstPressed();
            boolean backward = kGamePad1.isButtonBFirstPressed();
            if (!running && (forward || backward)) {
                runDistance = forward ? DISTANCE_MM : -DISTANCE_MM;
                move = new MoveStraightPIDAction(driveTrain, runDistance);
                move.setMaxPower(MAX_POWER);
                move.setToleranceMM(TOLERANCE_MM);
                move.setTimeoutMS(TIMEOUT_MS);
                start = SharedData.getOdometryWheelIMUPosition();
                peakOvershoot = 0;
                runTimer.reset();
                running = true;
                runCount++;
                double d = runDistance;
                KLog.d(TAG, () -> String.format("event=start dist=%.0f maxPower=%.2f tolMm=%.1f pXY=%.5f dXY=%.5f sXY=%.3f pAngle=%.4f",
                        d, MAX_POWER, TOLERANCE_MM, MoveStraightPIDAction.P_XY, MoveStraightPIDAction.D_XY,
                        MoveStraightPIDAction.S_XY, MoveStraightPIDAction.P_ANGLE));
            }

            if (running && kGamePad1.isButtonXFirstPressed()) {
                move.setIsDone(true); // stops the drivetrain
                lastResult = "cancelled";
                KLog.d(TAG, "event=cancelled");
            }

            Position current = SharedData.getOdometryWheelIMUPosition();
            double along = 0, cross = 0, headingErrDeg = 0;
            if (start != null) {
                // Error relative to the target, in the start-heading frame
                double h = start.getTheta();
                double dx = current.getX() - start.getX();
                double dy = current.getY() - start.getY();
                double traveled = dx * Math.cos(h) + dy * Math.sin(h);
                along = Math.signum(runDistance) * (traveled - runDistance); // + = past the target
                cross = -dx * Math.sin(h) + dy * Math.cos(h);
                headingErrDeg = Math.toDegrees(MathFunctions.angleWrapRad(current.getTheta() - h));
            }

            if (running && !move.getIsDone()) {
                move.updateCheckDone();
                peakOvershoot = Math.max(peakOvershoot, along);
                if (move.getIsDone() && !"cancelled".equals(lastResult)) {
                    boolean timedOut = runTimer.milliseconds() >= TIMEOUT_MS;
                    lastResult = String.format("#%d dist=%.0f %s in %.0f ms: along=%+.1f cross=%+.1f heading=%+.2f deg peakOvershoot=%.1f",
                            runCount, runDistance, timedOut ? "TIMED OUT" : "done", runTimer.milliseconds(),
                            along, cross, headingErrDeg, peakOvershoot);
                    String r = lastResult;
                    KLog.d(TAG, () -> "event=result " + r);
                }
            } else {
                if ("cancelled".equals(lastResult)) {
                    lastResult = "cancelled (run #" + runCount + ")";
                }
                driveAction.move(gamepad1);
            }

            telemetry.addLine("A/B: run fwd/back   dpad up/down: distance   dpad right/left: max power   X: cancel");
            telemetry.addData("Distance / max power", "%.0f mm / %.1f", DISTANCE_MM, MAX_POWER);
            telemetry.addData("State", running && !move.getIsDone() ? "RUNNING %.0f ms" : "idle", runTimer.milliseconds());
            if (start != null) {
                telemetry.addData("Along (+ past) / cross / heading", "%+.1f mm / %+.1f mm / %+.2f deg", along, cross, headingErrDeg);
            }
            telemetry.addData("Last result", lastResult);
            telemetry.addData("Gains (Dashboard: MoveStraightPIDAction)", "P_XY %.5f  D_XY %.5f  S_XY %.3f  P_ANGLE %.3f",
                    MoveStraightPIDAction.P_XY, MoveStraightPIDAction.D_XY, MoveStraightPIDAction.S_XY, MoveStraightPIDAction.P_ANGLE);
            telemetry.addData("Odometry", current);
            telemetry.update();
        }

        if (move != null && !move.getIsDone()) {
            move.setIsDone(true);
        }
        driveTrain.setPower(0);
        cleanupRobot();
    }
}
