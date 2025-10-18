package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.revolverActions.RevolverIntakeAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.navigation.PurePursuitAction;

public class IntakeCycle extends KActionSet {
    public static final double RED_MOVE_INCREMENT = 30;
    public IntakeCycle(DriveTrain driveTrain, Intake intake, Revolver revolver, TripleColorSensor colorSensors, double increment, double curX, double curY, double curHeading) {


        IntakeRun intakeRun = new IntakeRun(intake);
        intakeRun.setName("intakeRun");
        this.addAction(intakeRun);

//        PurePursuitAction move1 = new PurePursuitAction(driveTrain);
//        move1.setName("move1");
//        move1.setDependentActions(intakeRun);
//        move1.addPoint(curX, curY + increment, curHeading);
//        this.addAction(move1);


        RevolverIntakeAction revolverIntakeAction = new RevolverIntakeAction(revolver, colorSensors);
        revolverIntakeAction.setName("revolverIntakeAction");
//        revolverIntakeAction.setDependentActions();
        this.addAction(revolverIntakeAction);

//        PurePursuitAction move2 = new PurePursuitAction(driveTrain);
//        move2.setName("move2");
//        move2.setDependentActions(revolverIntakeAction);
//        move2.addPoint(curX, curY + increment*2, curHeading);
//        this.addAction(move2);

        RevolverIntakeAction revolverIntakeAction2 = new RevolverIntakeAction(revolver, colorSensors);
        revolverIntakeAction2.setName("revolverIntakeAction2");
        revolverIntakeAction2.setDependentActions(revolverIntakeAction);
        this.addAction(revolverIntakeAction2);

//        PurePursuitAction move3 = new PurePursuitAction(driveTrain);
//        move3.setName("move2");
//        move3.setDependentActions(revolverIntakeAction2);
//        move3.addPoint(curX, curY + increment*3, curHeading);
//        this.addAction(move3);

        RevolverIntakeAction revolverIntakeAction3 = new RevolverIntakeAction(revolver, colorSensors);
        revolverIntakeAction3.setName("revolverIntakeAction3");
        revolverIntakeAction3.setDependentActions(revolverIntakeAction2);
        this.addAction(revolverIntakeAction3);

        IntakeStop stop = new IntakeStop(intake);
        stop.setName("stop");
        stop.setDependentActions(revolverIntakeAction3);
        this.addAction(stop);




    }

}
