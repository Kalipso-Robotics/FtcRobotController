package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Simple intake check + pivot position finder. Both pivots start at ModuleConfig's DOWN positions. GP1:
 *   RT (hold)        = run both sides (power = trigger)
 *   LT (hold)        = reverse both sides
 *   LB               = select side A / B
 *   dpad up          = toggle side A motor at TEST_POWER (encoder / motor config check)
 *   dpad down        = toggle side B motor at TEST_POWER. X / B / RB sweep take over while active.
 *                      Check per side: +power must give + ticks and + RPS, and RPS ~ power * 27 (1620 rpm, no balls).
 *   dpad right/left  = step the selected pivot +/- PIVOT_STEP
 *   A / Y            = selected pivot to its config DOWN / UP
 *   X (hold)         = selected side closed loop at IntakeConfig.FEED_RPS (the shot feed), other side follows triggers
 *   B (hold)         = both sides closed loop, like PushBall's feed
 *   RB               = kF sweep (KfSweep), both sides open loop: 0.3, 0.5 ... 0.9, then 1.0, 10 s each.
 *                      Telemetry shows each step's RPS and kF (power / (RPS * voltageScale), averaged after
 *                      2 s of spin-up), then the average kF per side over all steps. RB again = cancel.
 *                      Triggers / X / B are ignored while it runs. Do it with NO balls in. Logged as event=sweep*.
 * Copy the found positions into IntakeConfig.SIDE_A / SIDE_B pivotDownPos / pivotUpPos.
 *
 * The stopper is held closed the whole OpMode, so balls stay in the system while the sides run.
 *
 * Feed velocity tuning, per side, in transfer motor RPS (gains are IntakeConfig.SIDE_A / SIDE_B kF, kP, live on Dashboard):
 *   1. kF, NO balls in the system: hold RT fully, read "kF est" for each side, copy into SIDE_A.kF / SIDE_B.kF.
 *      RPS at full power should be ~26 (1620 rpm motor); if not, fix ModuleConfig.INTAKE_TICKS_PER_REV first.
 *      Don't measure kF with balls against the closed stopper: the wheels near-stall and kF comes out far too high.
 *   2. Still no balls, hold X: RPS should sit near FEED_RPS with kP = 0. If not, fix kF first.
 *   3. Load 4 balls (stopper stays closed), hold X: "dip" is the lowest RPS since X went down. Raise kP until the dip
 *      shrinks without the speed hunting. Repeat for the other side (LB), then hold B to check both together.
 */
@Config
@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends KOpMode {
    public static final String TAG = "Test_Intake";
    public static double PIVOT_STEP = 0.01;
    public static double TEST_POWER = 0.5;
    private static final double FREE_RPS_AT_FULL_POWER = 1620 / 60.0; // goBILDA 1620 rpm, no load
    private static final double TRIGGER_DEADBAND = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        Intake intake = new Intake(opModeUtilities);
        IntakeModule sideA = intake.getModule(BeamBreakPosition.IntakeSide.A);
        IntakeModule sideB = intake.getModule(BeamBreakPosition.IntakeSide.B);
        intake.lower(); // init at ModuleConfig DOWN
        Stopper stopper = new Stopper(opModeUtilities); // constructor closes it; never opened here, so the 4 balls stay in
        boolean selectedA = true;
        boolean[] wasHolding = {false, false}; // [A, B]
        double[] dipRps = {0, 0};
        boolean[] testOn = {false, false}; // [A, B] dpad up / down
        KfSweep sweep = new KfSweep(TAG, "A", "B");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            updateActions(); // no actions here: refreshes SharedData voltage, which kF's voltageScale needs

            if (kGamePad1.isRightBumperFirstPressed()) {
                if (sweep.isRunning()) {
                    sweep.cancel();
                } else {
                    testOn[0] = testOn[1] = false; // the sweep owns both motors
                    sweep.start();
                }
            }

            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;
            double power = rt > TRIGGER_DEADBAND ? rt : lt > TRIGGER_DEADBAND ? -lt : 0;

            if (kGamePad1.isLeftBumperFirstPressed()) {
                selectedA = !selectedA;
            }
            boolean holdBoth = kGamePad1.isBPressed();
            boolean holdSelected = kGamePad1.isXPressed();
            IntakeModule[] sides = {sideA, sideB};
            if (kGamePad1.isDpadUpFirstPressed() && !sweep.isRunning()) {
                testOn[0] = !testOn[0];
                boolean on = testOn[0];
                KLog.d(TAG, () -> "side=A event=testPower on=" + on + " power=" + TEST_POWER);
            }
            if (kGamePad1.isDpadDownFirstPressed() && !sweep.isRunning()) {
                testOn[1] = !testOn[1];
                boolean on = testOn[1];
                KLog.d(TAG, () -> "side=B event=testPower on=" + on + " power=" + TEST_POWER);
            }
            if (sweep.isRunning()) {
                holdBoth = false; // the sweep owns both motors
                holdSelected = false;
                sweep.update(new double[]{sideA.getRPS(), sideB.getRPS()}, SharedData.getVoltage()); // RPS from last loop's power
                power = sweep.power(); // this step's power, 0 once done
            }
            for (int i = 0; i < 2; i++) {
                IntakeModule s = sides[i];
                boolean holding = holdBoth || (holdSelected && (i == 0) == selectedA);
                if (holding) {
                    s.runRPS(IntakeConfig.FEED_RPS);
                    double rps = s.getRPS();
                    // Dip ignores the spin-up: it only starts once the side first reaches 95% of the target.
                    if (!wasHolding[i]) {
                        dipRps[i] = Double.MAX_VALUE;
                    } else if (dipRps[i] != Double.MAX_VALUE || rps >= 0.95 * IntakeConfig.FEED_RPS) {
                        dipRps[i] = Math.min(dipRps[i], rps);
                    }
                    if (!wasHolding[i]) {
                        KLog.d(TAG, () -> "side=" + s.getSide() + " event=holdStart targetRps=" + IntakeConfig.FEED_RPS);
                    }
                } else {
                    s.run(testOn[i] ? TEST_POWER : power);
                    if (wasHolding[i]) {
                        double dip = dipRps[i];
                        KLog.d(TAG, () -> "side=" + s.getSide() + " event=holdEnd dipRps=" + dip);
                    }
                }
                wasHolding[i] = holding;
            }
            IntakeModule side = selectedA ? sideA : sideB;
            double down = side.getConfig().pivotDownPos;
            double up = side.getConfig().pivotUpPos;

            double step = 0;
            if (kGamePad1.isDpadRightFirstPressed()) step = PIVOT_STEP;
            else if (kGamePad1.isDpadLeftFirstPressed()) step = -PIVOT_STEP;

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

            telemetry.addLine("RT run | LT reverse | LB side | dpad U/D test A/B | R/L pivot +/-step | A down, Y up | X hold side, B hold both | RB kF sweep");
            for (int i = 0; i < 2; i++) {
                IntakeModule s = sides[i];
                double p = s.getPower();
                double rps = s.getRPS();
                String check = Math.abs(p) < 0.05 ? "motor off"
                        : Math.abs(rps) < 0.5 ? "NOT MOVING: encoder unplugged / wrong port, or motor stalled"
                        : Math.signum(rps) != Math.signum(p) ? "SIGN FLIPPED: encoder counts against power"
                        : "ok, RPS is " + Math.round(100 * Math.abs(rps) / (Math.abs(p) * FREE_RPS_AT_FULL_POWER)) + "% of expected";
                telemetry.addData("encoder " + s.getSide() + (testOn[i] ? " [TEST ON]" : ""),
                        "power %.2f | pos %d ticks | %.0f t/s | %.2f RPS (expect ~%.1f) | %s",
                        p, s.getCurrentPosition(), s.getVelocity(), rps, Math.abs(p) * FREE_RPS_AT_FULL_POWER, check);
            }
            sweep.addTelemetry(telemetry, new double[]{sideA.getRPS(), sideB.getRPS()});
            telemetry.addData("selected", selectedA ? "A (front)" : "B (back)");
            telemetry.addData("stopper", "CLOSED %.3f (balls only for the kP dip check)", stopper.getStopper().getTargetPosition());
            telemetry.addData("pivot step", "%.3f", PIVOT_STEP);
            telemetry.addData("triggers", "RT %.2f  LT %.2f -> power %.2f", rt, lt, power);
            telemetry.addData("feed target", "%.2f RPS  (0 = PushBall uses open loop)", IntakeConfig.FEED_RPS);
            if (IntakeConfig.FEED_RPS <= 0 || IntakeConfig.SIDE_A.kF <= 0 || IntakeConfig.SIDE_B.kF <= 0) {
                telemetry.addLine("!! X/B give 0 power until FEED_RPS and SIDE_A/SIDE_B kF are set (do the RT kF step first)");
            }
            for (int i = 0; i < 2; i++) {
                IntakeModule s = sides[i];
                IntakeModuleConfig cfg = s.getConfig();
                double rps = s.getRPS();
                if (wasHolding[i]) {
                    double feedTarget = IntakeConfig.FEED_RPS;
                    double dip = dipRps[i] == Double.MAX_VALUE ? rps : dipRps[i];
                    telemetry.addData("hold " + s.getSide(), "kF %.5f kP %.5f | %.2f RPS  err %.1f%%  dip %.2f (%.1f%%)  power %.2f",
                            cfg.kF, cfg.kP,
                            rps, feedTarget > 0 ? 100 * (feedTarget - rps) / feedTarget : 0,
                            dip, feedTarget > 0 ? 100 * (feedTarget - dip) / feedTarget : 0, s.getPower());
                } else {
                    telemetry.addData("kF est " + s.getSide(), "%.5f  (no balls, RT full, open loop)", s.estimateKf());
                }
                telemetry.addData("side " + s.getSide(), "pivot %.3f (down %.2f up %.2f) %s | power %.2f  %.0f mA  %.2f RPS",
                        s.getPivotPosition(),
                        cfg.pivotDownPos, cfg.pivotUpPos,
                        s.isDown() ? "DOWN" : s.isUp() ? "UP" : "moving/custom",
                        s.getPower(), s.getCurrentMA(), rps);
            }
            telemetry.addData("flower", "side %s  %s", ModuleConfig.FLOWER_SIDE,
                    intake.getFlower().isDown() ? "DOWN" : intake.getFlower().isUp() ? "UP" : "moving");
            telemetry.update();
        }
        intake.stop();
        cleanupRobot();
    }
}
