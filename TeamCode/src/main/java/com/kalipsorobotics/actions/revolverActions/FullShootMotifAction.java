package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.shooter.ShooterReady;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class FullShootMotifAction extends KActionSet {
    public FullShootMotifAction(Revolver revolver, Shooter shooter, MotifCamera.MotifPattern motifPattern, TripleColorSensor colorSensors, OpModeUtilities opModeUtilities) {

        DetectColorsAction detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        ShooterReady shooterReady = new ShooterReady(shooter, Shooter.RED_TARGET_FROM_NEAR, LaunchPosition.AUTO); //todo set point
        shooterReady.setName("shooterReady");
        this.addAction(shooterReady);

        RevolverShootColorAction revolverShootColorAction1 = new RevolverShootColorAction(revolver, shooter, motifPattern.first, detectColorsAction);
        revolverShootColorAction1.setName("revolverShootColorAction1");
        this.addAction(revolverShootColorAction1);
        revolverShootColorAction1.setDependentActions(shooterReady);

        WaitAction waitAction1 = new WaitAction(300);
        waitAction1.setName("waitAction");
        this.addAction(waitAction1);
        waitAction1.setDependentActions(revolverShootColorAction1);

        RevolverShootColorAction revolverShootColorAction2 = new RevolverShootColorAction(revolver, shooter, motifPattern.middle, detectColorsAction);
        revolverShootColorAction2.setName("revolverShootColorAction2");
        revolverShootColorAction2.setDependentActions(revolverShootColorAction1, waitAction1);
        this.addAction(revolverShootColorAction2);

        WaitAction waitAction2 = new WaitAction(300);
        waitAction2.setName("waitAction");
        this.addAction(waitAction2);
        waitAction2.setDependentActions(revolverShootColorAction2);

        RevolverShootColorAction revolverShootColorAction3 = new RevolverShootColorAction(revolver, shooter, motifPattern.last, detectColorsAction);
        revolverShootColorAction3.setName("revolverShootColorAction3");
        revolverShootColorAction3.setDependentActions(revolverShootColorAction2, waitAction2);
        this.addAction(revolverShootColorAction3);

        WaitAction waitAction3 = new WaitAction(300);
        waitAction3.setName("waitAction");
        this.addAction(waitAction3);
        waitAction3.setDependentActions(revolverShootColorAction3);

        ShooterStop stop = new ShooterStop(shooter);
        stop.setName("stop");
        stop.setDependentActions(revolverShootColorAction3, waitAction3);
        this.addAction(stop);

    }
}
