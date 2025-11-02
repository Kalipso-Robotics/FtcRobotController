package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Pusher;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class FullShootMotifAction extends KActionSet {
    public FullShootMotifAction(Revolver revolver, Shooter shooter, Pusher pusher, Intake intake, MotifCamera.MotifPattern motifPattern, TripleColorSensor colorSensors, OpModeUtilities opModeUtilities) {

        DetectColorsAction detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        ShooterReady shooterReady1 = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO); //todo set point
        shooterReady1.setName("shooterReady1");
        this.addAction(shooterReady1);

        RevolverShootColorAction revolverShootColorAction1 = new RevolverShootColorAction(revolver, pusher, intake, motifPattern.first, detectColorsAction);
        revolverShootColorAction1.setName("revolverShootColorAction1");
        this.addAction(revolverShootColorAction1);
        revolverShootColorAction1.setDependentActions(shooterReady1);

        ShooterReady shooterReady2 = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO); //todo set point
        shooterReady2.setName("shooterReady2");
        this.addAction(shooterReady2);
        shooterReady2.setDependentActions(revolverShootColorAction1);

        RevolverShootColorAction revolverShootColorAction2 = new RevolverShootColorAction(revolver, pusher, intake, motifPattern.middle, detectColorsAction);
        revolverShootColorAction2.setName("revolverShootColorAction2");
        revolverShootColorAction2.setDependentActions(revolverShootColorAction1, shooterReady2);
        this.addAction(revolverShootColorAction2);

        ShooterReady shooterReady3 = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO); //todo set point
        shooterReady3.setName("shooterReady3");
        this.addAction(shooterReady3);
        shooterReady2.setDependentActions(revolverShootColorAction2);

        RevolverShootColorAction revolverShootColorAction3 = new RevolverShootColorAction(revolver, pusher, intake, motifPattern.last, detectColorsAction);
        revolverShootColorAction3.setName("revolverShootColorAction3");
        revolverShootColorAction3.setDependentActions(revolverShootColorAction2, shooterReady3);
        this.addAction(revolverShootColorAction3);


        ShooterStop stop = new ShooterStop(shooter);
        stop.setName("stop");
        stop.setDependentActions(revolverShootColorAction3);
        this.addAction(stop);

    }
}
