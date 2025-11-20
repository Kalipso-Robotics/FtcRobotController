package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.shooter.ShooterRun;
import com.kalipsorobotics.actions.shooter.ShooterStop;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Stopper;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.shooter.LaunchPosition;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class FullShootMotifAction extends KActionSet {
    public FullShootMotifAction(Revolver revolver, Shooter shooter, Stopper stopper, Intake intake, MotifCamera.MotifPattern motifPattern, TripleColorSensor colorSensors, OpModeUtilities opModeUtilities) {

        DetectColorsAction detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        ShooterRun shooterRun1 = new ShooterRun(shooter, Shooter.RED_TARGET_FROM_FAR, LaunchPosition.AUTO); //todo set point
        shooterRun1.setName("shooterReady1");
        this.addAction(shooterRun1);

        RevolverShootColorAction revolverShootColorAction1 = new RevolverShootColorAction(shooter, revolver, stopper, intake, motifPattern.first, detectColorsAction);
        revolverShootColorAction1.setName("revolverShootColorAction1");
        this.addAction(revolverShootColorAction1);
        revolverShootColorAction1.setDependentActions(shooterRun1);

        ShooterRun shooterRun2 = new ShooterRun(shooter, Shooter.RED_TARGET_FROM_FAR, LaunchPosition.AUTO); //todo set point
        shooterRun2.setName("shooterReady2");
        this.addAction(shooterRun2);
        shooterRun2.setDependentActions(revolverShootColorAction1);

        RevolverShootColorAction revolverShootColorAction2 = new RevolverShootColorAction(shooter, revolver, stopper, intake, motifPattern.middle, detectColorsAction);
        revolverShootColorAction2.setName("revolverShootColorAction2");
        revolverShootColorAction2.setDependentActions(revolverShootColorAction1, shooterRun2);
        this.addAction(revolverShootColorAction2);

        ShooterRun shooterRun3 = new ShooterRun(shooter, Shooter.RED_TARGET_FROM_FAR, LaunchPosition.AUTO); //todo set point
        shooterRun3.setName("shooterReady3");
        this.addAction(shooterRun3);
        shooterRun2.setDependentActions(revolverShootColorAction2);

        RevolverShootColorAction revolverShootColorAction3 = new RevolverShootColorAction(shooter, revolver, stopper, intake, motifPattern.last, detectColorsAction);
        revolverShootColorAction3.setName("revolverShootColorAction3");
        revolverShootColorAction3.setDependentActions(revolverShootColorAction2, shooterRun3);
        this.addAction(revolverShootColorAction3);

        ShooterStop stop = new ShooterStop(shooterRun3);
        stop.setName("stop");
        stop.setDependentActions(revolverShootColorAction3);
        this.addAction(stop);

    }
}
