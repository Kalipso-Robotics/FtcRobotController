package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_X;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_Y;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

/*
175 1145 111
65 1145 109
265 1185 170
25 1200 170
130 780 90
SHOOT_FAR_X SHOOT_FAR_Y
 */


public class DepotRoundTrip extends KActionSet {

    RoundTripAction moveToShoot;
    IPurePursuitAction moveToDepot;
    IntakeFullAction intakeBalls;
    double intakeTimeMS = IntakeConfig.intakeBallTimeMS;
    public DepotRoundTrip(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, Point target, Point launchPos, double waitForShooterReadyMS, AllianceColor allianceColor, boolean useAdaptivePP) {
        turretAutoAlign.setToleranceDeg(1.5);

        if (!useAdaptivePP) {
            moveToDepot = new PurePursuitAction(drivetrain);
        } else {
            moveToDepot = new AdaptivePurePursuitAction(drivetrain);
        }
        moveToDepot.setName("moveToDepot");
        moveToDepot.setMaxTimeOutMS(4000);
        moveToDepot.setLookAheadRadius(150);
        moveToDepot.setFinalSearchRadiusMM(150);
        moveToDepot.setFinalAngleLockingThresholdDeg(40);
        moveToDepot.setPathAngleToleranceDeg(40);
        moveToDepot.addPoint(100, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        moveToDepot.addPoint(100, 1100 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        this.addAction(moveToDepot);

        intakeBalls = new IntakeFullAction(stopper, intake, intakeTimeMS, 1);
        intakeBalls.setName("intakeBalls");
        this.addAction(intakeBalls);

        moveToShoot = new RoundTripAction(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, target, launchPos, waitForShooterReadyMS, true, true);
        moveToShoot.setName("moveToShoot");
        moveToShoot.getMoveToBall().setLookAheadRadius(150);
        moveToShoot.getMoveToBall().setMaxTimeOutMS(1800);
        moveToShoot.getMoveToBall().setFinalSearchRadiusMM(150);
        moveToShoot.getMoveToBall().setFinalAngleLockingThresholdDeg(30);
        moveToShoot.getPurePursuitReadyShooting().setDistanceThresholdMM(100);
        moveToShoot.getPurePursuitReadyIntakeStop().setDistanceThresholdMM(800);
        moveToShoot.getMoveToBall().setPathAngleToleranceDeg(10);
        moveToShoot.setDependentActions(moveToDepot);
//        trip.getPushBall().getRunUntilFullSpeed().setFullSpeedDurationMs(200);
        moveToShoot.setShouldShooterStop(false);
        //trip.getMoveToBall().addPoint(15, 600 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        //trip.getMoveToBall().addPoint(15, 900 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        //trip.getMoveToBall().addPoint(15,  1168 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        moveToShoot.getMoveToBall().addPoint(SHOOT_FAR_X, SHOOT_FAR_Y * allianceColor.getPolarity() , 90 * allianceColor.getPolarity());
        this.addAction(moveToShoot);
    }

    public DepotRoundTrip(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, Point target, Point launchPos, double waitForShooterReadyMS, AllianceColor allianceColor) {
        this(opModeUtilities, drivetrain, turretAutoAlign, shooter, stopper, intake, target, launchPos, waitForShooterReadyMS, allianceColor, false);
    }

    @Override
    protected void afterUpdate() {
        if (moveToShoot.getPurePursuitReadyIntakeStop().getIsDone()) {
            intakeBalls.stopAndSetDone();
        }
    }

    public RoundTripAction getMoveToShoot() {
        return moveToShoot;
    }

    public IPurePursuitAction getMoveToDepot() {
        return moveToDepot;
    }

    public void setIntakeTimeMS(double intakeTimeMS) {
        this.intakeTimeMS = intakeTimeMS;
    }
}
