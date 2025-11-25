package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.actions.intake.IntakeRunFullSpeed;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.intake.RunIntakeUntilFullSpeed;
import com.kalipsorobotics.actions.turret.TurretAutoAlign;
import com.kalipsorobotics.actions.turret.TurretConfig;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Turret;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.KTeleOp;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Regression Model Data Collector")
public class ShooterRegressionModelDataCollector extends KTeleOp {
    KFileWriter kFileWriter = null;
    Shooter shooter = null;
    Intake intake = null;
    IntakeRunFullSpeed intakeRunFullSpeed = null;
    IntakeStop intakeStop = null;
    RunIntakeUntilFullSpeed runUntilFullSpeed = null;
    TurretAutoAlign turretAutoAlign = null;
    Stopper stopper = null;
    KServoAutoAction stop = null;
//    ShootAllAction shootAction = null;
    OpModeUtilities opModeUtilities = null;
    Turret turret = null;
    private double rps = 26.2;
    private double hoodPosition = 0.55;
    private double distance = 0;
    private boolean isIncrementRps = true;
    private double hoodIncrementation = 0.05;
    private double rpsIncrementation = 0.5;
    @Override
    public void initializeRobot() {
        super.initializeRobot(); // Initialize base class components including kGamePad1
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        kFileWriter = new KFileWriter("shooter_data", opModeUtilities); //log destination
        shooter = new Shooter(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        intake = new Intake(opModeUtilities);
        intakeRunFullSpeed = new IntakeRunFullSpeed(intake);
        intakeStop = new IntakeStop(intake);
        runUntilFullSpeed = new RunIntakeUntilFullSpeed(intake);
        turret = Turret.getInstance(opModeUtilities);
        turretAutoAlign = new TurretAutoAlign(opModeUtilities, turret, TurretConfig.X_INIT_SETUP, TurretConfig.Y_INIT_SETUP);
//        shootAction = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
        kFileWriter.writeLine("distance, rps, hood position");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        waitForStart();
        while (opModeIsActive()) {
            /*
                BUTTON A: LOG FILE
                BUTTON B: SHOOT

                BUTTON X: SWITCH CURRENT INCREMENT (RPS & HOOD POSITION)

                RIGHT BUMPER: increment current selected incrementation +
                RIGHT TRIGGER: increment current selected incrementation -

                LEFT BUMPER: increment current selected incrementation + bigi
                LEFT TRIGGER: increment current selected incrementation - big

                hood max is 0.55

                rps: 26.2
                hood: 0.55
             */

            //TODO make csv converter into array for easy coding
            distance = shooter.getDistance(SharedData.getOdometryPosition(), Shooter.TARGET_POINT);
            if (kGamePad1.isButtonAFirstPressed()) { //log
                kFileWriter.writeLine(distance + "," + rps + "," + hoodPosition);
                KLog.d("Regression Module Data Collector", "data logged: distance: " + distance + " rps: " + rps + " hood pos: " + hoodPosition);
            }


            if (kGamePad1.isButtonXFirstPressed()) {
                if (isIncrementRps) {
                    isIncrementRps = false;
                } else {
                    isIncrementRps = true;
                }
            }
            if (kGamePad1.isRightBumperPressed()) {
                if (isIncrementRps) {
                    rps += rpsIncrementation;
                } else {
                    hoodPosition += hoodIncrementation;
                }
            }
            if (kGamePad1.isRightTriggerFirstPressed()) {
                if (isIncrementRps) {
                    rps -= rpsIncrementation;
                    rps = Math.max(0, rps);
                } else {
                    hoodPosition -= hoodIncrementation;
                }
            }

            if (kGamePad1.isLeftBumperPressed()) {
                if (isIncrementRps) {
                    rps += (rpsIncrementation*5);
                    rps = Math.max(0, rps);
                } else {
                    hoodPosition += (hoodIncrementation*5);
                }
            }
            if (kGamePad1.isLeftTriggerFirstPressed()) {
                if (isIncrementRps) {
                    rps -= (rpsIncrementation*5);
                } else {
                    hoodPosition -= (hoodIncrementation * 5);
                }
            }
            if (kGamePad1.isYPressed()) {
                // Create new intake action each time to ensure it runs
                if (intakeRunFullSpeed != null || intakeRunFullSpeed.getIsDone()) {
                    intakeRunFullSpeed = new IntakeRunFullSpeed(intake);
                    stop = new KServoAutoAction(stopper.getStopper(), stopper.STOPPER_SERVO_CLOSED_POS);
                    setLastStopperAction(stop);
                    setLastIntakeAction(intakeRunFullSpeed);
                }
            } else {
                if (intakeStop != null || intakeStop.getIsDone()) {
                    intakeStop = new IntakeStop(intake);
                    setLastIntakeAction(intakeStop);
                }
            }
            if (kGamePad1.isDpadDownFirstPressed()) {
                shooter.stop();
            }

            if (kGamePad1.isDpadUpFirstPressed()) {
                if (runUntilFullSpeed != null || runUntilFullSpeed.getIsDone()) {
                    runUntilFullSpeed = new RunIntakeUntilFullSpeed(intake);
                    setLastIntakeAction(runUntilFullSpeed);
                }
                stopper.getStopper().setPosition(0.7);
            }

//            if (kGamePad1.isDpadLeftFirstPressed()) {
//                stopper.getStopper().setPosition(0.7);
//            }

            if (isIncrementRps) {
                telemetry.addData("Current Increment: ", "RPS");
            } else {
                telemetry.addData("Current Increment: ", "Hood Position");
            }

            if (kGamePad1.isButtonBFirstPressed()) { //shoot
                shooter.getHood().setPosition(hoodPosition);
                shooter.goToRPS(rps);
                KLog.d("Regression Module Data Collector", "Shooter Ready set");
                //shootAction = new ShootAllAction(stopper, intake, shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO);
                KLog.d("Regression Module Data Collector", "ball shot");

            }


            telemetry.addData("RPS: ", rps);
            telemetry.addData("Hood Position", hoodPosition);
            telemetry.update();

            turretAutoAlign.updateCheckDone();
            // Update all actions
            updateActions();
        }
        kFileWriter.close();
    }
}
