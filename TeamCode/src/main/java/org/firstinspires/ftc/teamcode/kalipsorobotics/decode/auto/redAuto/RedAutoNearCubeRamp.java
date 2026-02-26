package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.auto.redAuto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActions.pathActions.RoundTripAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;


@Disabled
public class RedAutoNearCubeRamp extends RedAutoNearRamp {

    @Override
    protected void handleTrip1() {
        trip1 = new RoundTripAction(opModeUtilities, driveTrain, turretAutoAlign, shooter, stopper, intake, Shooter.TARGET_POINT.multiplyY(allianceColor.getPolarity()), nearLaunchPoint.multiplyY(allianceColor.getPolarity()), 0);
        trip1.setName("trip1");
        trip1.getShooterReady().setName("shooterReady_trip1");
        trip1.getMoveToBall().clearPoints();
        trip1.getMoveToBall().addPoint(1350, 225 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(1350, 960 * allianceColor.getPolarity(), 90 * allianceColor.getPolarity());
        // move to lever
        trip1.getMoveToBall().addPoint(1600, 1060 * allianceColor.getPolarity(), 180 * allianceColor.getPolarity());
        // move to launch
        trip1.getMoveToBall().addPoint(1500, nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip1.getMoveToBall().addPoint(nearLaunchPoint.getX(), nearLaunchPoint.getY() * allianceColor.getPolarity(), 150 * allianceColor.getPolarity());
        trip1.setDependentActions(trip0);
        trip1.setShouldShooterStop(false);
        trip1.getMoveToBall().setLookAheadRadius(200);
        trip1.getPurePursuitReadyShooting().setDistanceThresholdMM(350);
        trip1.getMoveToBall().setFinalAngleLockingThresholdDeg(50);
        redAutoNear.addAction(trip1);
    }

}
