package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.cameraVision.MotifCamera;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.modules.TripleColorSensor;
import com.kalipsorobotics.modules.shooter.Shooter;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class FullShootMotifAction extends KActionSet {
    public FullShootMotifAction(Revolver revolver, Shooter shooter, MotifCamera.MotifPattern motifPattern, TripleColorSensor colorSensors, OpModeUtilities opModeUtilities) {

        DetectColorsAction detectColorsAction = new DetectColorsAction(colorSensors, opModeUtilities);

        RevolverShootColorAction revolverShootColorAction1 = new RevolverShootColorAction(revolver, shooter, motifPattern.first, detectColorsAction);
        revolverShootColorAction1.setName("revolverShootColorAction1");
        this.addAction(revolverShootColorAction1);

        RevolverShootColorAction revolverShootColorAction2 = new RevolverShootColorAction(revolver, shooter, motifPattern.middle, detectColorsAction);
        revolverShootColorAction2.setName("revolverShootColorAction2");
        revolverShootColorAction2.setDependentActions(revolverShootColorAction1);
        this.addAction(revolverShootColorAction2);

        RevolverShootColorAction revolverShootColorAction3 = new RevolverShootColorAction(revolver, shooter, motifPattern.last, detectColorsAction);
        revolverShootColorAction3.setName("revolverShootColorAction3");
        revolverShootColorAction3.setDependentActions(revolverShootColorAction2);
        this.addAction(revolverShootColorAction3);
    }
}
