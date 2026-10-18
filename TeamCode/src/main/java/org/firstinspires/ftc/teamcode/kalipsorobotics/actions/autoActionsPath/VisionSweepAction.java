package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.cameraVision.VisionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.ClusterDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

public class VisionSweepAction extends KActionSet {

    public VisionSweepAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, AllianceColor allianceColor, boolean useAdaptivePP) {
        VisionAction vision = new VisionAction(() -> SharedData.getBallInformation(), 1);
        vision.setName("vision");
        this.addAction(vision);

        ClusterDetectionAction clusterDetectionAction = new ClusterDetectionAction(SharedData.getBallInformation(), new Point(SharedData.getOdometryWheelIMUPosition().getX(), SharedData.getOdometryWheelIMUPosition().getY()), allianceColor);
        clusterDetectionAction.setDependentActions(vision);
        clusterDetectionAction.setName("clusterDetectionAction");
        this.addAction(clusterDetectionAction);

        AutoToBallsPathingAction autoToBallsPathingAction = new AutoToBallsPathingAction(drivetrain, stopper, intake, true);
        autoToBallsPathingAction.setDependentActions(clusterDetectionAction);
        autoToBallsPathingAction.setName("autoToBallsPathingAction");
        this.addAction(autoToBallsPathingAction);
    }


}
