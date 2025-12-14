package com.kalipsorobotics.decode;

import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import com.kalipsorobotics.actions.intake.IntakeFullAction;
import com.kalipsorobotics.actions.intake.IntakeStop;
import com.kalipsorobotics.actions.shooter.stopper.CloseStopperAction;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.navigation.PurePursuitAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.dataflow.qual.Pure;

import java.nio.channels.ClosedByInterruptException;

@Autonomous(name="RedAutoNearTwoLeverPush")
public class RedAutoNearTwoLeverPush extends RedAutoNear{
    @Override
    public void handleTrip3() {
        //just purepsuit - not a round trip
        PurePursuitAction trip2_1 = new PurePursuitAction(driveTrain);
        trip2_1.setName("trip2.5");
        trip2_1.setDependentActions(trip2);
        trip2_1.setFinalSearchRadius(50);
        trip2_1.addPoint(1047.13, 461.97, 50 * allianceColor.getPolarity());
        trip2_1.addPoint(1359.49, 1098.34, 50 * allianceColor.getPolarity());
        redAutoNear.addAction(trip2_1);

        CloseStopperAction closeStopperAction = new CloseStopperAction(stopper);
        closeStopperAction.setName("closeStopper");
        closeStopperAction.setDependentActions(trip2);
        redAutoNear.addAction(closeStopperAction);

        IntakeFullAction intakeFullAction = new IntakeFullAction(stopper, intake, 5000);
        intakeFullAction.setName("intake");
        redAutoNear.addAction(intakeFullAction);
        intakeFullAction.setDependentActions(trip2_1);

        WaitAction waitAction = new WaitAction(1000);
        waitAction.setName("waitForIntake");
        waitAction.setDependentActions(trip2_1);
        redAutoNear.addAction(waitAction);

        trip3 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), new Point(THIRD_SHOOT_NEAR_X, THIRD_SHOOT_NEAR_Y * allianceColor.getPolarity()), 500);
        trip3.setName("trip3");
        trip3.setDependentActions(trip2_1);
        trip3.getMoveToBall().clearPoints();
        trip3.getMoveToBall().addPoint(THIRD_SHOOT_NEAR_X, THIRD_SHOOT_NEAR_Y * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        redAutoNear.addAction(trip3);
    }
}
