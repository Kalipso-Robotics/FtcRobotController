package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.intakeFromRampTime;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper.CloseStopperAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;


public class RampCycleAction extends KActionSet {
    RoundTripAction tripToShoot = null;
    PurePursuitAction moveToRamp;
    PurePursuitAction moveToEat;
    OpModeUtilities opModeUtilities;
    IntakeFullAction intakeFullAction;
    DriveTrain driveTrain;
    TurretAutoAlign turretAutoAlign;
    Shooter shooter;
    Stopper stopper;
    Intake intake;
    Point targetPoint;
    Point launchPos;
    double waitForShooterReadyMS;
    double waitTimeAfterMoveToEat; // wait for intake
    double waitForGate;

    ShooterRun shooterRun;

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS, double waitTimeAfterMoveToEat, double waitForGate) {
        this.opModeUtilities = opModeUtilities;
        this.driveTrain = driveTrain;
        this.turretAutoAlign = turretAutoAlign;
        this.shooter = shooter;
        this.stopper = stopper;
        this.intake = intake;
        this.targetPoint = targetPoint;
        this.launchPos = launchPos;
        this.waitForShooterReadyMS = waitForShooterReadyMS;
        this.waitTimeAfterMoveToEat = waitTimeAfterMoveToEat;
        this.waitForGate = waitForGate;

        moveToRamp = new PurePursuitAction(driveTrain);
        moveToRamp.setName("rampCycleTrip1");
        moveToRamp.setPathAngleTolerance(10);
        moveToRamp.setFinalSearchRadius(100);
        moveToRamp.setFinalAngleLockingThresholdDegree(10);
        moveToRamp.setMaxTimeOutMS(3000);
        this.addAction(moveToRamp);

        moveToEat = new PurePursuitAction(driveTrain);
        moveToEat.setName("rampCycleTrip2");
        moveToEat.setPathAngleTolerance(10);
        moveToEat.setFinalSearchRadius(100);
        moveToEat.setFinalAngleLockingThresholdDegree(10);
        moveToRamp.setMaxTimeOutMS(2000);
        this.addAction(moveToEat);

        tripToShoot = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake,targetPoint, launchPos, 500);
        tripToShoot.setName("rampCycleTrip3");
        tripToShoot.getMoveToBall().clearPoints();
        tripToShoot.getMoveToBall().setFinalSearchRadius(200);
        tripToShoot.getMoveToBall().setMaxTimeOutMS(4000);
        tripToShoot.getMoveToBall().setFinalAngleLockingThresholdDegree(45);
        tripToShoot.getMoveToBall().setPathAngleTolerance(50);
        this.addAction(tripToShoot);
    }

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS) {
        this(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, targetPoint, launchPos, waitForShooterReadyMS, 1000, 0);
    }

    public RampCycleAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPos, double waitForShooterReadyMS, double waitTimeAfterMoveToEat) {
        this(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, targetPoint, launchPos, waitForShooterReadyMS, waitTimeAfterMoveToEat, 0);
    }

    @Override
    public void initActions(){

        WaitAction waitUntilShootRun = new WaitAction(waitForShooterReadyMS);
        waitUntilShootRun.setName("waitUntilShootReady");
        this.addAction(waitUntilShootRun);

        //x=1405.56 (55.34 in), y=-1201.27 (-47.29 in), theta=-0.8324 (-4
        CloseStopperAction closeStopperAction = new CloseStopperAction(stopper);
        closeStopperAction.setName("closeStopper");
        this.addAction(closeStopperAction);

        shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, launchPos);
        shooterRun.setName("shooterRun");
        shooterRun.setDependentActions();
        this.addAction(shooterRun);

        intakeFullAction = new IntakeFullAction(stopper, intake, intakeFromRampTime, 1);
        intakeFullAction.setName("intake");
        intakeFullAction.setDependentActions(moveToRamp);
        this.addAction(intakeFullAction);

//        WaitAction waitGate = new WaitAction(waitForGate);
//        waitGate.setName("waitForGate");
//        waitGate.setDependentActions(moveToRamp);
//        this.addAction(waitGate);

        moveToEat.setDependentActions(moveToRamp);

        WaitAction waitActionAfterMoveToEat = new WaitAction(waitTimeAfterMoveToEat);
        waitActionAfterMoveToEat.setName("waitForIntake");
        waitActionAfterMoveToEat.setDependentActions(moveToEat);
        this.addAction(waitActionAfterMoveToEat);

        tripToShoot.setDependentActions(waitActionAfterMoveToEat);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(tripToShoot);
        this.addAction(shooterStop);
    }

    public PurePursuitAction getMoveToRamp() {
        return moveToRamp;
    }

    public PurePursuitAction getMoveToEat() {
        return moveToEat;
    }

    public RoundTripAction getTripToShoot() {
        return tripToShoot;
    }

    @Override
    protected void beforeUpdate() {
        super.beforeUpdate();

        KLog.d("RampCycle", () -> String.format("[%s] Status - MoveToRamp: %s, MoveToEat: %s, Intake: %s, ShooterRun: %s, TripToShoot: %s",
                getName() != null ? getName() : "unnamed",
                moveToRamp.getIsDone() ? "DONE" : "NOT DONE",
                moveToEat.getIsDone() ? "DONE" : "NOT DONE",
                intakeFullAction.getIsDone() ? "DONE" : "NOT DONE",
                shooterRun.getIsDone() ? "DONE" : "NOT DONE",
                tripToShoot.getIsDone() ? "DONE" : "NOT DONE"));
    }

    @Override
    public void afterUpdate() {
        if (moveToRamp.getIsDone()) {
            intakeFullAction.setIsDone(true);
            KLog.d("RampCycle", () -> String.format("[%s] IntakeFullAction stopped by MoveToRamp completion",
                    getName() != null ? getName() : "unnamed"));
        }
    }

}
