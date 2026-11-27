package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.ThreeTipSoloAuto.FLOWER_EXIT_POSITION_1;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.ThreeTipSoloAuto.FLOWER_INTAKE_POSITION_1;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.FullFlowerAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeDecisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeSources;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
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
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PathPlanning;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

import java.util.List;

/**
 * One trip, written like a trip in RedThreeTipSoloAuto:
 *   intake (where IntakeDecisionAction says) -> go to shoot -> shoot -> stop shooter.
 *
 * The decision is made in the constructor, so build this trip only when the robot
 * is ready to go (after the last trip is done), not before the match starts.
 * If there is nowhere left to intake, the trip has no actions and is done right away.
 *
 * The first drive of each intake (to the flower, the garden or a tip zone) is planned around
 * the Hive and the flowers by PathPlanning, starting from where the robot is when the trip is built.
 */
public class DecisionRoundTripAction extends KActionSet {

    private final DriveTrain driveTrain;
    private final Stopper stopper;
    private final Intake intake;
    private final FlowerMech flowerMech;
    private final double flowerIntakeMS;

    private Point robot;            // where the robot is now, before this trip
    private PathPlanning planner;   // the Hive, the flowers and the midline as obstacles

    public DecisionRoundTripAction(OpModeUtilities opModeUtilities, DriveTrain driveTrain, Shooter shooter,
                                   Stopper stopper, Intake intake, FlowerMech flowerMech,
                                   TurretAutoAlign turretAutoAlign, ShootDecisionAction shootDecision,
                                   IntakeDecisionAction decision, double flowerIntakeMS) {
        this.driveTrain = driveTrain;
        this.stopper = stopper;
        this.intake = intake;
        this.flowerMech = flowerMech;
        this.flowerIntakeMS = flowerIntakeMS;

        decision.decide();   // also runs shootDecision.decide()
        Position shootPosition = shootDecision.getShootPosition();   // side of the Cell that is up
        Point targetPoint = shootDecision.getTargetPoint();          // the Cell that is up
        IntakeSources whereToIntake = decision.getChosenSource();
        if (whereToIntake == null) {
            return;   // nowhere has balls: empty trip
        }
        robot = SharedData.getOdometryWheelIMUPosition().toPoint();

        // Hive never seen: act like the rigid auto. Flowers go to Cell B, everything after goes to Cell A.
        if (SharedData.getHiveUpCell() == null
                && whereToIntake != IntakeSources.FLOWER_1 && whereToIntake != IntakeSources.FLOWER_2) {
            shootPosition = ShootDecisionAction.SHOOT_TIP_ZONE_2;
            targetPoint = ShooterConfig.TARGET_POINT_1;
        }

        // INTAKE
        KActionSet intakeBalls = new KActionSet();
        intakeBalls.setName("intakeBalls");
        planner = PathPlanning.forFieldAuto(AutoToBallsPathingAction.BUFFER_MM,
                RobotFootprint.WIDTH_MM / 2 + AutoToBallsPathingAction.AUTO_MIDLINE_MARGIN_MM);
        if (whereToIntake == IntakeSources.FLOWER_1) {
            addFlowerIntake(intakeBalls);
        } else if (whereToIntake == IntakeSources.FLOWER_2) {
            addFlowerIntake(intakeBalls);
        } else if (whereToIntake == IntakeSources.GARDEN) {
            addGardenIntake(intakeBalls);
        } else {
            addTipZoneIntake(intakeBalls, whereToIntake);
        }
        this.addAction(intakeBalls);

        // GO TO SHOOT, then SHOOT
        addShoot(opModeUtilities, shooter, turretAutoAlign, intakeBalls, shootPosition, targetPoint);
    }

    /** Flowers: a planned drive to the setup point, then FullFlowerAction drives in, intakes and backs out by itself. */

    //TODO FIX logic. ONLY FLOWER A CURRENTLY
    private void addFlowerIntake(KActionSet intakeBalls) {
        // A straight line from the start to Flower 1 hits the Hive leg, so this drive is planned.
        List<Point> route = planner.plan(robot, FLOWER_INTAKE_POSITION_1.toPoint());   // null = no route found

        IPurePursuitAction goToFlowerSetup = new AdaptivePurePursuitAction(driveTrain);
        goToFlowerSetup.setName("goToFlowerSetup");
        goToFlowerSetup.setMaxTimeOutMS(4500); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        if (route != null) {
            for (int i = 0; i < route.size() - 1; i++) {
                Point corner = route.get(i);
                goToFlowerSetup.addPosition(corner.getX(), corner.getY(), FLOWER_INTAKE_POSITION_1.getTheta());
            }
        }
        goToFlowerSetup.addPosition(FLOWER_INTAKE_POSITION_1);
        intakeBalls.addAction(goToFlowerSetup);

        FullFlowerAction flowerIntake = new FullFlowerAction(stopper, intake, flowerIntakeMS, IntakeConfig.intakePower, flowerMech, driveTrain, FLOWER_INTAKE_POSITION_1, FLOWER_EXIT_POSITION_1);
        flowerIntake.setName("flowerIntake");
        flowerIntake.setDependentActions(goToFlowerSetup);   // its own setup move is now a tiny move
        intakeBalls.addAction(flowerIntake);
    }

    /** Garden: a planned drive to the garden entry, then the same sweep as RedThreeTipSoloAuto trip 3, intake running. */
    private void addGardenIntake(KActionSet intakeBalls) {
        IPurePursuitAction goToGarden = new AdaptivePurePursuitAction(driveTrain);
        goToGarden.setName("goToGarden");
        goToGarden.setMaxTimeOutMS(4500); // give up if blocked, so the robot never keeps pushing (TODO-measure)

        Point gardenEntry = new Point(289, 2949);
        List<Point> gardenRoute = planner.plan(robot, gardenEntry);   // null = no route found
        if (gardenRoute != null) {
            for (int i = 0; i < gardenRoute.size() - 1; i++) {
                Point corner = gardenRoute.get(i);
                goToGarden.addPosition(corner.getX(), corner.getY(), 102.6);
            }
        }
        goToGarden.addPosition(gardenEntry.getX(), gardenEntry.getY(), 102.6);
        intakeBalls.addAction(goToGarden);

        IntakeFullAction runIntake = new IntakeFullAction(stopper, intake, IntakeConfig.intakeBallTimeMS, IntakeConfig.intakePower);
        runIntake.setName("runIntake");
        runIntake.setDependentActions(goToGarden);
        intakeBalls.addAction(runIntake);

        // The sweep is an exact line along the garden on purpose, so it is NOT planned.
        IPurePursuitAction sweepGarden = new AdaptivePurePursuitAction(driveTrain);
        sweepGarden.setName("sweepGarden");
        sweepGarden.setMaxTimeOutMS(3000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        sweepGarden.addPosition(188, 3309, 90.0);
        sweepGarden.addPosition(289, 2949, 102.6);
        sweepGarden.addPosition(188, 3309, 90.0);
        sweepGarden.setDependentActions(goToGarden);
        intakeBalls.addAction(sweepGarden);

        // Sweep done: stop the intake now, so its timer does not keep it running into the shot (same as the rigid auto).
        IntakeStop stopIntake = new IntakeStop(runIntake);
        stopIntake.setName("stopIntake");
        stopIntake.setDependentActions(sweepGarden);
        intakeBalls.addAction(stopIntake);
    }

    /** Tip zones: a planned drive to the look spot, then vision finds the balls and drives through them. */
    private void addTipZoneIntake(KActionSet intakeBalls, IntakeSources tipZone) {
        Point intakePoint = IntakeDecisionAction.pointOf(tipZone);
        double headingDeg = IntakeDecisionAction.headingDegOf(tipZone);

        IPurePursuitAction goToIntake = new AdaptivePurePursuitAction(driveTrain);
        goToIntake.setName("goToIntake");
        goToIntake.setMaxTimeOutMS(4000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        List<Point> tipRoute = planner.plan(robot, intakePoint);   // null = no route found (tip zone 1 is inside Flower A's buffer)
        if (tipRoute != null) {
            for (int i = 0; i < tipRoute.size() - 1; i++) {
                Point corner = tipRoute.get(i);
                goToIntake.addPosition(corner.getX(), corner.getY(), headingDeg);
            }
        }
        goToIntake.addPosition(intakePoint.getX(), intakePoint.getY(), headingDeg);
        intakeBalls.addAction(goToIntake);

        VisionPickupAction lookAndGrabBalls = new VisionPickupAction(driveTrain, stopper, intake, AllianceColor.RED);
        lookAndGrabBalls.setName("lookAndGrabBalls");
        lookAndGrabBalls.setTimeoutMs(6000);
        lookAndGrabBalls.setDependentActions(goToIntake);   // look only after arriving at the look spot
        intakeBalls.addAction(lookAndGrabBalls);
    }

    /** Drive to the shoot spot after the intake, spin up, aim, shoot, stop the shooter. */
    private void addShoot(OpModeUtilities opModeUtilities, Shooter shooter, TurretAutoAlign turretAutoAlign,
                          KActionSet intakeBalls, Position shootPosition, Point targetPoint) {
        IPurePursuitAction goToShoot = new AdaptivePurePursuitAction(driveTrain);
        goToShoot.setName("goToShoot");
        goToShoot.setMaxTimeOutMS(4000); // give up if blocked, so the robot never keeps pushing (TODO-measure)
        goToShoot.addPosition(shootPosition.getX(), shootPosition.getY(), Math.toDegrees(shootPosition.getTheta()));
        goToShoot.setDependentActions(intakeBalls);
        this.addAction(goToShoot);

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
