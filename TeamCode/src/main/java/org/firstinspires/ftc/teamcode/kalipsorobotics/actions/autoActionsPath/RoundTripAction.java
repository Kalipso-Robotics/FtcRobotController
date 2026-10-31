package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_X;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_Y;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReadyAuto;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

public class RoundTripAction extends KActionSet {
    IPurePursuitAction moveToShoot;

    //shoot action and stuff

    IntakeFullAction intakeBalls;
    IPurePursuitAction moveToBalls;

    double intakeTimeMS = 1000;


    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, /* TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, */ AllianceColor allianceColor, boolean useAdaptivePP) {
        //turretAutoAlign.setToleranceDeg(1.5);

        if (!useAdaptivePP) {
            moveToShoot = new PurePursuitAction(drivetrain);
        } else {
            moveToShoot = new AdaptivePurePursuitAction(drivetrain);
        }
        moveToShoot.setName("moveToShoot");
        this.addAction(moveToShoot);

        // shoot stuff

        if (!useAdaptivePP) {
            moveToBalls = new PurePursuitAction(drivetrain);
        } else {
            AdaptivePurePursuitAction adaptiveMoveToBalls = new AdaptivePurePursuitAction(drivetrain);
            adaptiveMoveToBalls.setPlanStartFrom(moveToShoot);
            moveToBalls = adaptiveMoveToBalls;
        }
        moveToBalls.setName("moveToBalls");
        moveToBalls.setDependentActions(moveToShoot);
        this.addAction(moveToBalls);

//        intakeBalls = new IntakeFullAction(stopper, intake, intakeTimeMS, 1);
//        intakeBalls.setName("intakeBalls");
//        intakeBalls.setDependentActions(moveToBalls);
//        this.addAction(intakeBalls);
    }

    /**
     * Same trip as above, with the shooting and intake switched on: drive to the shoot point, shoot at the target Cell, then drive
     * to the balls with the intake running (a Garden or a Flower is picked up by driving into it). The intake stops when the
     * drive to the balls ends.
     */
    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake,
                           Point targetPoint, Point launchPoint, AllianceColor allianceColor, boolean useAdaptivePP) {
        this(opModeUtilities, drivetrain, allianceColor, useAdaptivePP);

        // the flywheel warms up while the robot drives to the shoot point
        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, launchPoint);
        shooterRun.setName("shooterRun");
        this.addAction(shooterRun);

        ShooterReady shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        shooterReady.setDependentActions(moveToShoot);
        this.addAction(shooterReady);

        TurretReadyAuto turretReady = new TurretReadyAuto(turretAutoAlign);
        turretReady.setName("turretReady");
        turretReady.setDependentActions(moveToShoot);
        this.addAction(turretReady);

        TurretStop turretStop = new TurretStop(turretAutoAlign);
        turretStop.setName("turretStop");
        turretStop.setDependentActions(turretReady);
        this.addAction(turretStop);

        PushBall pushBall = new PushBall(stopper, intake);
        pushBall.setName("shoot");
        pushBall.setDependentActions(moveToShoot, turretReady);
        this.addAction(pushBall);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(pushBall);
        this.addAction(shooterStop);

        // after the shot: drive to the balls and intake on the way
        moveToBalls.setDependentActions(moveToShoot, pushBall);

        intakeBalls = new IntakeFullAction(stopper, intake, 10000, IntakeConfig.intakePower);
        intakeBalls.setName("intakeBalls");
        intakeBalls.setDependentActions(pushBall);
        this.addAction(intakeBalls);

        IntakeStop intakeStop = new IntakeStop(intakeBalls);
        intakeStop.setName("intakeStop");
        intakeStop.setDependentActions(moveToBalls);
        this.addAction(intakeStop);
    }

    public IPurePursuitAction getMoveToShoot() {return moveToShoot;}
    public IPurePursuitAction getMoveToBalls() {return moveToBalls;}

}
