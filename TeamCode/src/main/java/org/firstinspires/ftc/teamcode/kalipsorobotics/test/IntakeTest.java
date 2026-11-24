package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;

/**
 * Simple intake check + pivot position finder. Both pivots start at ModuleConfig's DOWN positions. GP1:
 *   RT (hold)        = run both sides (power = trigger)
 *   LT (hold)        = reverse both sides
 *   LB               = select side A / B
 *   dpad up / down   = step the selected pivot +/- PIVOT_STEP
 *   dpad right/left  = step the selected pivot +/- PIVOT_STEP * 10
 *   A / Y            = selected pivot to ModuleConfig DOWN / UP
 * Copy the found positions into ModuleConfig.INTAKE_PIVOT_*_POS.
 */
@Config
@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends KOpMode {
    public static final String TAG = "Test_Intake";
    public static double PIVOT_STEP = 0.01;
    private static final double TRIGGER_DEADBAND = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Intake intake = new Intake(opModeUtilities);
        IntakeSide sideA = intake.getSide(BeamBreakPosition.IntakeSide.A);
        IntakeSide sideB = intake.getSide(BeamBreakPosition.IntakeSide.B);
        intake.lower(); // init at ModuleConfig DOWN
        boolean selectedA = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();

            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;
            double power = rt > TRIGGER_DEADBAND ? rt : lt > TRIGGER_DEADBAND ? -lt : 0;
            intake.run(power);

            if (kGamePad1.isLeftBumperFirstPressed()) {
                selectedA = !selectedA;
            }
            IntakeSide side = selectedA ? sideA : sideB;
            double down = selectedA ? ModuleConfig.INTAKE_PIVOT_A_DOWN_POS : ModuleConfig.INTAKE_PIVOT_B_DOWN_POS;
            double up = selectedA ? ModuleConfig.INTAKE_PIVOT_A_UP_POS : ModuleConfig.INTAKE_PIVOT_B_UP_POS;

            double step = 0;
            if (kGamePad1.isDpadUpFirstPressed()) step = PIVOT_STEP;
            else if (kGamePad1.isDpadDownFirstPressed()) step = -PIVOT_STEP;
            else if (kGamePad1.isDpadRightFirstPressed()) step = PIVOT_STEP * 10;
            else if (kGamePad1.isDpadLeftFirstPressed()) step = -PIVOT_STEP * 10;

            Double target = null;
            if (step != 0) target = KServo.clampServoPos(side.getPivotPosition() + step);
            else if (kGamePad1.isButtonAFirstPressed()) target = down;
            else if (kGamePad1.isButtonYFirstPressed()) target = up;
            if (target != null) {
                boolean ok = side.setPivotPosition(target); // false = flower interlock refused
                double t = target;
                KLog.d(TAG, () -> "side=" + side.getSide() + " event=pivot pos=" + t + " accepted=" + ok);
            }
            intake.update();

            telemetry.addLine("RT run | LT reverse | LB side | dpad U/D +/-step, R/L +/-10x | A down, Y up");
            telemetry.addData("selected", selectedA ? "A (front)" : "B (back)");
            telemetry.addData("pivot step", "%.3f", PIVOT_STEP);
            telemetry.addData("triggers", "RT %.2f  LT %.2f -> power %.2f", rt, lt, power);
            for (IntakeSide s : new IntakeSide[]{sideA, sideB}) {
                boolean isA = s == sideA;
                telemetry.addData("side " + s.getSide(), "pivot %.3f (down %.2f up %.2f) %s | power %.2f  %.0f mA  %.0f t/s",
                        s.getPivotPosition(),
                        isA ? ModuleConfig.INTAKE_PIVOT_A_DOWN_POS : ModuleConfig.INTAKE_PIVOT_B_DOWN_POS,
                        isA ? ModuleConfig.INTAKE_PIVOT_A_UP_POS : ModuleConfig.INTAKE_PIVOT_B_UP_POS,
                        s.isDown() ? "DOWN" : s.isUp() ? "UP" : "moving/custom",
                        s.getPower(), s.getCurrentMA(), s.getVelocity());
            }
            telemetry.addData("flower", "side %s  %s", ModuleConfig.FLOWER_SIDE,
                    intake.getFlower().isDown() ? "DOWN" : intake.getFlower().isUp() ? "UP" : "moving");
            telemetry.update();
        }
        intake.stop();
        cleanupRobot();
    }
}
