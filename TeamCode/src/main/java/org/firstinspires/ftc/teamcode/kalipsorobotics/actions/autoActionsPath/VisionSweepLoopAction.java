package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import com.qualcomm.robotcore.util.ElapsedTime;

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

public class VisionSweepLoopAction extends KActionSet {

    ElapsedTime autoTimer;

    public VisionSweepLoopAction(OpModeUtilities opModeUtilities, DriveTrain drivetrain, TurretAutoAlign turretAutoAlign, Shooter shooter, Stopper stopper, Intake intake, ElapsedTime autoTimer, AllianceColor allianceColor, boolean useAdaptivePP) {
        this.autoTimer = autoTimer;

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

    @Override
    public void update() {
        super.update();

        double timeToPark = 0; //todo calculation

        if (timeToPark >= 30000-autoTimer.milliseconds()) {
            isDone = true;
        } else {
            isDone = false;
        }

        // decision will update and return a ???

        // then need to readjust plan based on decision
    }


}
