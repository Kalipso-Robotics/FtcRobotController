package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.Flower;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FullFlowerAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeDecisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeSources;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootDecisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * One trip, written like a trip in RedThreeTipSoloAuto:
 *   intake (where IntakeDecisionAction says) -> go to shoot -> shoot -> stop shooter.
 *
 * The decision is made in the constructor, so build this trip only when the robot
 * is ready to go (after the last trip is done), not before the match starts.
 * If there is nowhere left to intake, the trip has no actions and is done right away.
 */
public class DecisionRoundTripAction extends KActionSet {

    public DecisionRoundTripAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, Shooter shooter,
                                   Stopper stopper, Intake intake, FlowerMech flowerMech,
                                   TurretAutoAlign turretAutoAlign, ShootDecisionAction shootDecision,
                                   IntakeDecisionAction decision, double flowerIntakeMS) {
        decision.decide();   // also runs shootDecision.decide()
        Position shootPosition = shootDecision.getShootPosition();   // side of the Cell that is up
        Point targetPoint = shootDecision.getTargetPoint();          // the Cell that is up
        IntakeSources whereToIntake = decision.getChosenSource();
        if (whereToIntake == null) {
            return;   // nowhere has balls: empty trip
        }

        // Hive never seen: act like the rigid auto. Flowers go to Cell B, everything after goes to Cell A.
        if (SharedData.getHiveUpCell() == null
                && whereToIntake != IntakeSources.FLOWER_1 && whereToIntake != IntakeSources.FLOWER_2) {
            shootPosition = ShootDecisionAction.SHOOT_TIP_ZONE_2;
            targetPoint = ShooterConfig.TARGET_POINT_1;
        }

        // INTAKE
        KActionSet intakeBalls = new KActionSet();
        intakeBalls.setName("intakeBalls");
        if (whereToIntake == IntakeSources.FLOWER_1 || whereToIntake == IntakeSources.FLOWER_2) {
            // flowers: FullFlowerAction drives in, intakes and backs out by itself
            Flower flower;
            if (whereToIntake == IntakeSources.FLOWER_1) {
                flower = Flower.FLOWER_1;
            } else {
                flower = Flower.FLOWER_2;
            }
            FullFlowerAction flowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, flower);
            flowerIntake.setName("flowerIntake");
            intakeBalls.addAction(flowerIntake);
        } else if (whereToIntake == IntakeSources.GARDEN) {
            // garden: same path as RedThreeTipSoloAuto trip 3. The intake runs while driving and sweeping.
            IPurePursuitAction goToGarden = new PurePursuitAction(driveTrain);
            goToGarden.setName("goToGarden");
            goToGarden.addPoint(1027, 487, 108.5);
            goToGarden.addPoint(289, 2949, 102.6);
            intakeBalls.addAction(goToGarden);

            IntakeFullAction runIntake = new IntakeFullAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower);
            runIntake.setName("runIntake");
            runIntake.setDependentActions(goToGarden);
            intakeBalls.addAction(runIntake);

            IPurePursuitAction sweepGarden = new PurePursuitAction(driveTrain);
            sweepGarden.setName("sweepGarden");
            sweepGarden.addPoint(188, 3309, 90.0);
            sweepGarden.addPoint(289, 2949, 102.6);
            sweepGarden.addPoint(188, 3309, 90.0);
            sweepGarden.setDependentActions(goToGarden);
            intakeBalls.addAction(sweepGarden);
        } else {
            // tip zones: drive there with the intake on
            Point intakePoint = IntakeDecisionAction.pointOf(whereToIntake);

            IPurePursuitAction goToIntake = new PurePursuitAction(driveTrain);
            goToIntake.setName("goToIntake");
            goToIntake.addPoint(intakePoint.getX(), intakePoint.getY(), IntakeDecisionAction.headingDegOf(whereToIntake));
            intakeBalls.addAction(goToIntake);

            IntakeFullAction runIntake = new IntakeFullAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower);
            runIntake.setName("runIntake");
            runIntake.setDependentActions(goToIntake);
            intakeBalls.addAction(runIntake);
        }
        this.addAction(intakeBalls);

        // GO TO SHOOT
        IPurePursuitAction goToShoot = new PurePursuitAction(driveTrain);
        goToShoot.setName("goToShoot");
        goToShoot.addPoint(shootPosition.getX(), shootPosition.getY(), Math.toDegrees(shootPosition.getTheta()));
        goToShoot.setDependentActions(intakeBalls);
        this.addAction(goToShoot);

        // SHOOT
        ShooterRun shooterRun = new ShooterRun(opModeUtilities, shooter, targetPoint, shootPosition.toPoint());
        shooterRun.setName("shooterRun");
        shooterRun.setDependentActions(goToShoot);
        this.addAction(shooterRun);

        ShooterReady shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        shooterReady.setDependentActions(goToShoot);
        this.addAction(shooterReady);

        TurretReady turretReady = new TurretReady(turretAutoAlign);
        turretReady.setName("turretReady");
        turretReady.setDependentActions(goToShoot);
        this.addAction(turretReady);

        PushBall shoot = new PushBall(stopper, intake);
        shoot.setName("shoot");
        shoot.setDependentActions(shooterReady, turretReady);
        this.addAction(shoot);

        ShooterStop shooterStop = new ShooterStop(shooterRun);
        shooterStop.setName("shooterStop");
        shooterStop.setDependentActions(shoot);
        this.addAction(shooterStop);
    }
}
