package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_X;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto.RedAutoDepot.SHOOT_FAR_Y;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeFullAction;
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

    double intakeTimeMS = IntakeConfig.intakeBallTimeMS;


    public RoundTripAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, AllianceColor allianceColor, boolean useAdaptivePP) {
        turretAutoAlign.setToleranceDeg(1.5);

        if (!useAdaptivePP) {
            moveToShoot = new PurePursuitAction(drivetrain);
        } else {
            moveToShoot = new AdaptivePurePursuitAction(drivetrain);
        }
        moveToShoot.setName("moveToShoot");
        this.addAction(moveToShoot);

        // shoot stuff

        intakeBalls = new IntakeFullAction(stopper, intake, intakeTimeMS, 1);
        intakeBalls.setName("intakeBalls");
        this.addAction(intakeBalls);

        if (!useAdaptivePP) {
            moveToBalls = new PurePursuitAction(drivetrain);
        } else {
            moveToBalls = new AdaptivePurePursuitAction(drivetrain);
        }
        moveToBalls.setName("moveToBalls");
        moveToBalls.setDependentActions(moveToShoot);
        this.addAction(moveToBalls);
    }

    public IPurePursuitAction getMoveToShoot() {return moveToShoot;}
    public IPurePursuitAction getMoveToBalls() {return moveToBalls;}

}
