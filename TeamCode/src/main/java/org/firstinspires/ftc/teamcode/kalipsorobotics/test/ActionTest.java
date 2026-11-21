package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FlowerLower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FlowerRaise;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeSideLower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeSideRaise;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStow;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.RunIntakeTime;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.HoodSetAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper.CloseStopperAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper.OpenStopperAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretSetAngle;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;

/**
 * One button per auto action, to check each on the robot before it goes into an auto.
 * Pattern source: decode/calibrations/StopperCalibration (KOpMode + setLast*Action).
 *
 * GP1 (RB released):  dpad down/up = lower/raise A   dpad left/right = lower/raise B   A = stow
 *                     X = flower down   Y = flower up   LB = stopper open   LT = stopper close
 *                     B = PushBall   RT = IntakeFullAction A 2 s   back = RunIntakeTime B reversed 1 s
 * GP1 (RB held):      dpad down = hood MIN   dpad left/right = hood mid   dpad up = hood MAX
 *                     X = turret -90   Y = turret 0   B = turret +90
 */
@TeleOp(name = "ActionTest", group = "Test")
public class ActionTest extends KOpMode {
    public static final String TAG = "Test_Action";

    private Intake intake;
    private IntakeSide sideA;
    private IntakeSide sideB;
    private Stopper stopper;
    private Shooter shooter;
    private Turret turret;
    private Action running = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        intake = new Intake(opModeUtilities);
        sideA = intake.getSide(BeamBreakPosition.IntakeSide.A);
        sideB = intake.getSide(BeamBreakPosition.IntakeSide.B);
        stopper = new Stopper(opModeUtilities);
        shooter = new Shooter(opModeUtilities);
        Turret.setInstanceNull();
        turret = Turret.getInstance(opModeUtilities);

        telemetry.addLine("ActionTest: one button per action. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            if (gamepad1.right_bumper) {
                secondLayer();
            } else {
                firstLayer();
            }
            intake.update();
            updateActions();
            renderTelemetry();
        }
        intake.stop();
        shooter.stop();
        cleanupRobot();
    }

    private void firstLayer() {
        if (kGamePad1.isDpadDownFirstPressed()) {
            startIntake(new IntakeSideLower(sideA), "lowerA");
        } else if (kGamePad1.isDpadUpFirstPressed()) {
            startIntake(new IntakeSideRaise(sideA), "raiseA");
        } else if (kGamePad1.isDpadLeftFirstPressed()) {
            startIntake(new IntakeSideLower(sideB), "lowerB");
        } else if (kGamePad1.isDpadRightFirstPressed()) {
            startIntake(new IntakeSideRaise(sideB), "raiseB");
        } else if (kGamePad1.isButtonAFirstPressed()) {
            startIntake(new IntakeStow(intake), "stow");
        } else if (kGamePad1.isButtonXFirstPressed()) {
            startIntake(new FlowerLower(intake.getFlower(), 1000), "flowerDown");
        } else if (kGamePad1.isButtonYFirstPressed()) {
            startIntake(new FlowerRaise(intake.getFlower()), "flowerUp");
        } else if (kGamePad1.isLeftBumperFirstPressed()) {
            startStopper(new OpenStopperAction(stopper), "stopperOpen");
        } else if (kGamePad1.isLeftTriggerFirstPressed()) {
            startStopper(new CloseStopperAction(stopper), "stopperClose");
        } else if (kGamePad1.isButtonBFirstPressed()) {
            startShooter(new PushBall(stopper, intake), "pushBall");
        } else if (kGamePad1.isRightTriggerFirstPressed()) {
            startIntake(new IntakeFullAction(stopper, sideA, 2000, IntakeConfig.intakePower), "intakeFullA2s");
        } else if (kGamePad1.isBackButtonFirstPressed()) {
            startIntake(new RunIntakeTime(sideB, 1000, -IntakeConfig.intakePower), "reverseB1s");
        }
    }

    private void secondLayer() {
        double mid = (ShooterInterpolationConfig.MIN_HOOD + ShooterInterpolationConfig.MAX_HOOD) / 2;
        if (kGamePad1.isDpadDownFirstPressed()) {
            startShooter(new HoodSetAction(shooter, ShooterInterpolationConfig.MIN_HOOD), "hoodMin");
        } else if (kGamePad1.isDpadLeftFirstPressed() || kGamePad1.isDpadRightFirstPressed()) {
            startShooter(new HoodSetAction(shooter, mid), "hoodMid");
        } else if (kGamePad1.isDpadUpFirstPressed()) {
            startShooter(new HoodSetAction(shooter, ShooterInterpolationConfig.MAX_HOOD), "hoodMax");
        } else if (kGamePad1.isButtonXFirstPressed()) {
            startMove(new TurretSetAngle(turret, -90), "turretMinus90");
        } else if (kGamePad1.isButtonYFirstPressed()) {
            startMove(new TurretSetAngle(turret, 0), "turret0");
        } else if (kGamePad1.isButtonBFirstPressed()) {
            startMove(new TurretSetAngle(turret, 90), "turretPlus90");
        }
    }

    private void startIntake(Action action, String name) {
        name(action, name);
        setLastIntakeAction(action);
    }

    private void startStopper(Action action, String name) {
        name(action, name);
        setLastStopperAction(action);
    }

    private void startShooter(Action action, String name) {
        name(action, name);
        setLastShooterAction(action);
    }

    private void startMove(Action action, String name) {
        name(action, name);
        setLastMoveAction(action);
    }

    private void name(Action action, String name) {
        action.setName(name);
        running = action;
        KLog.d(TAG, () -> "event=start action=" + name);
    }

    private void renderTelemetry() {
        telemetry.addLine("RB up:   dpad dn/up lower/raise A | dpad L/R lower/raise B | A stow");
        telemetry.addLine("         X flower down | Y flower up | LB stopper open | LT close");
        telemetry.addLine("         B PushBall | RT IntakeFull A 2s | back B reversed 1s");
        telemetry.addLine("RB held: dpad dn/L,R/up hood MIN/mid/MAX | X/Y/B turret -90/0/+90");
        if (running == null) {
            telemetry.addData("action", "none yet");
        } else {
            telemetry.addData("action", "%s  %s", running.getName(), running.getIsDone() ? "DONE" : "running");
        }
        telemetry.addData("pivots", "A %.3f %s  B %.3f %s", sideA.getPivotPosition(), sideA.isDown() ? "down" : "up",
                sideB.getPivotPosition(), sideB.isDown() ? "down" : "up");
        telemetry.addData("flower", "%.3f", intake.getFlower().getPosition());
        telemetry.addData("stopper", "%.3f", stopper.getStopper().getTargetPosition());
        telemetry.addData("hood", "%.4f", shooter.getHood().getTargetPosition());
        telemetry.addData("turret", "%.1f deg", turret.getAngleDeg());
        telemetry.update();
    }
}
