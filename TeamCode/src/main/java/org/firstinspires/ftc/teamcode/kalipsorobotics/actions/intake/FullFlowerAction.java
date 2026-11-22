package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.IPurePursuitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.PurePursuitAction;

public class FullFlowerAction extends KActionSet {
    public FullFlowerAction(Stopper stopper, Intake intake, double timeMS, double power, FlowerMech flowerMech, DriveTrain driveTrain, Flower flower){

        IPurePursuitAction setup = new PurePursuitAction(driveTrain);
        setup.addPoint(flower.getSetupPoint().getX(), flower.getSetupPoint().getY(), flower.getSetupHeadingDeg());
        setup.setName("setup");
        this.addAction(setup);

        IPurePursuitAction goToFlower = new PurePursuitAction(driveTrain);
        goToFlower.addPoint(flower.getPoint().getX(), flower.getPoint().getY(), flower.getFlowerHeadingDeg());
        goToFlower.setName("goToFlower");
        goToFlower.setDependentActions(setup);
        this.addAction(goToFlower);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper,intake,timeMS,power);
        intakeFullAction.setName("intakeFullAction");
        intakeFullAction.setDependentActions(setup);
        this.addAction(intakeFullAction);

        FlowerLower flowerLower = new FlowerLower(flowerMech, flower.getTimeoutMS());
        flowerLower.setName("flowerLower");
        flowerLower.setDependentActions(setup);
        this.addAction(flowerLower);

        IPurePursuitAction exitFlower = new PurePursuitAction(driveTrain);
        exitFlower.addPoint(flower.getExitPoint().getX(), flower.getExitPoint().getY(), flower.getExitHeadingDeg());
        exitFlower.setName("exitFlower");
        exitFlower.setDependentActions(goToFlower);
        this.addAction(exitFlower);

        FlowerRaise flowerRaise = new FlowerRaise(flowerMech);
        flowerRaise.setName("flowerRaise");
        flowerRaise.setDependentActions(exitFlower);
        this.addAction(flowerRaise);

    }
}
