package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.MoveStraightPIDAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.AdaptivePurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.sim.RedTippingAuto;

/**
 * One Flower pile: drive to the setup point, intake on and Flower arm down, drive onto the flower, back out to
 * the exit point once the arm is down, then stop the intake and raise the arm.
 *
 * 
 */
public class FullFlowerAction extends KActionSet {
    // How far the straight move pushes into the flower after goToFlower (backward: the flower is behind the robot).
    public static final double PUSH_IN_MM = 260;
    private double intakeTimeMS = 1250;


    public IPurePursuitAction getGoToFlower() {
        return goToFlower;
    }

    public void setGoToFlower(IPurePursuitAction goToFlower) {
        this.goToFlower = goToFlower;
    }

    private IPurePursuitAction goToFlower;

    public KServoAutoAction getFlowerLower() {
        return flowerLower;
    }

    public void setFlowerLower(KServoAutoAction flowerLower) {

        this.flowerLower = flowerLower;
    }

    public void setIntakeTime(double ms) {
        this.intakeTimeMS = ms;
    }

    private KServoAutoAction flowerLower;
    public FullFlowerAction(Stopper stopper, Intake intake, double intakeTimeMS, double power, FlowerMech flowerMech, DriveTrain driveTrain, Position flowerIntakePosition, Position flowerExitPosition) {
        // (0, 0) is the field corner, never a real setup or exit point. Until they are measured in Flower.java this
        // trip is skipped (the set has no actions, so it is done at once) so the rest of the auto can still be tested.

        goToFlower = new AdaptivePurePursuitAction(driveTrain);
        goToFlower.setMaxTimeOutMS(5000); // give up if blocked (e.g. against the flower base) (TODO-measure)
        goToFlower.addPosition(flowerIntakePosition);
        goToFlower.setName("goToFlower");
        this.addAction(goToFlower);

        MoveStraightPIDAction moveStraightToIntake = new MoveStraightPIDAction(driveTrain, -PUSH_IN_MM); // negative = backward: at 90 deg the flower is behind the robot (lower y)
        moveStraightToIntake.setTimeoutMS(1500);
        moveStraightToIntake.setName("moveStraightToIntake");
        moveStraightToIntake.setDependentActions(goToFlower);
        this.addAction(moveStraightToIntake);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper, intake, 1_000_000, power);
        intakeFullAction.setName("intakeFullAction");
        intakeFullAction.setDependentActions();
        this.addAction(intakeFullAction);

//        flowerLower = new KServoAutoAction(flowerMech.getServo(), FlowerMech.DOWN_POS);
//        flowerLower.setName("flowerLower");
//        this.addAction(flowerLower);


        WaitAction waitAction = new WaitAction(this.intakeTimeMS);
        waitAction.setName("waitForFlower");
        waitAction.setDependentActions(moveStraightToIntake); // not goToFlower: exitFlower must not drive while the straight move still is
        this.addAction(waitAction);

        // Back out only once the arm is down (or gave up), so the robot never leaves before the arm swept the pile.
//        AdaptivePurePursuitAction exitFlower = new AdaptivePurePursuitAction(driveTrain);
//        // The straight move moved the robot PUSH_IN_MM back from goToFlower's end: plan from there, so this path is
//        // planned during init instead of replanned (robot sitting still) during the match.
//        double h = flowerIntakePosition.getTheta();
//        exitFlower.setPlanStartPosition(new Position(flowerIntakePosition.getX() - PUSH_IN_MM * Math.cos(h),
//                flowerIntakePosition.getY() - PUSH_IN_MM * Math.sin(h), h));
//        exitFlower.setMaxTimeOutMS(1500); // give up if blocked (e.g. against the flower base) (TODO-measure)
//        exitFlower.addPosition(flowerExitPosition);
//        exitFlower.setName("exitFlower");
//        exitFlower.setDependentActions(waitAction);
//        this.addAction(exitFlower);

        // Out of the flower: stop the intake now instead of letting its timer run into the next shot.
        IntakeStop stopIntake = new IntakeStop(intakeFullAction);
        stopIntake.setName("stopIntake");
        stopIntake.setDependentActions(waitAction);
        this.addAction(stopIntake);


    }

    private static boolean isPlaceholder(Point point) {
        return point.getX() == 0 && point.getY() == 0;
    }
}
