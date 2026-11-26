package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * One Flower pile: drive to the setup point, intake on and Flower arm down, drive onto the flower, back out to
 * the exit point once the arm is down, then stop the intake and raise the arm.
 *
 * @param intakeTimeMS longest the intake runs (it is also stopped as soon as the robot has backed out),
 *                     so keep it short: the drive in and out, not IntakeConfig.intakeBallTimeMS
 */
public class FullFlowerAction extends KActionSet {
    public FlowerLower getFlowerLower() {
        return flowerLower;
    }

    public void setFlowerLower(FlowerLower flowerLower) {

        this.flowerLower = flowerLower;
    }

    private FlowerLower flowerLower;
    public FullFlowerAction(Stopper stopper, Intake intake, double intakeTimeMS, double power, FlowerMech flowerMech, DriveTrain driveTrain, Flower flower){
        // (0, 0) is the field corner, never a real setup or exit point. Until they are measured in Flower.java this
        // trip is skipped (the set has no actions, so it is done at once) so the rest of the auto can still be tested.

        if (isPlaceholder(flower.getSetupPoint()) || isPlaceholder(flower.getExitPoint())) {
            KLog.e("FullFlowerAction", flower + " skipped: setup/exit point is still the (0, 0) placeholder, measure it in Flower.java");
            return;
        }

        IPurePursuitAction goToFlower = new AdaptivePurePursuitAction(driveTrain);
        goToFlower.setMaxTimeOutMS(4000); // give up if blocked (e.g. against the flower base) (TODO-measure)
        goToFlower.addPoint(flower.getPoint().getX(), flower.getPoint().getY(), flower.getFlowerHeadingDeg());
        goToFlower.setName("setup");
        this.addAction(goToFlower);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper, intake, intakeTimeMS, power);
        intakeFullAction.setName("intakeFullAction");
        this.addAction(intakeFullAction);

        flowerLower = new FlowerLower(flowerMech, flower.getTimeoutMS());
        flowerLower.setName("flowerLower");
        this.addAction(flowerLower);


        WaitAction waitAction = new WaitAction(2000);
        waitAction.setName("waitForFlower");
        waitAction.setDependentActions(goToFlower);
        this.addAction(waitAction);

        // Back out only once the arm is down (or gave up), so the robot never leaves before the arm swept the pile.
        IPurePursuitAction exitFlower = new AdaptivePurePursuitAction(driveTrain);
        exitFlower.setMaxTimeOutMS(1500); // give up if blocked (e.g. against the flower base) (TODO-measure)
        exitFlower.addPoint(flower.getExitPoint().getX(), flower.getExitPoint().getY(), flower.getExitHeadingDeg());
        exitFlower.setName("exitFlower");
        exitFlower.setDependentActions(waitAction);
        this.addAction(exitFlower);

        // Out of the flower: stop the intake now instead of letting its timer run into the next shot.
        IntakeStop stopIntake = new IntakeStop(intakeFullAction);
        stopIntake.setName("stopIntake");
        stopIntake.setDependentActions(exitFlower);
        this.addAction(stopIntake);

        FlowerRaise flowerRaise = new FlowerRaise(flowerMech);
        flowerRaise.setName("flowerRaise");
        flowerRaise.setDependentActions(exitFlower);
        this.addAction(flowerRaise);
    }

    private static boolean isPlaceholder(Point point) {
        return point.getX() == 0 && point.getY() == 0;
    }
}
