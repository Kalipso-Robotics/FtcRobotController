package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.DriveBrake;

public class ActivateBraking extends KActionSet {

    private final double BRAKE_RIGHT_POS = 0.8135;
    private final double BRAKE_LEFT_POS = 0.7195;

    public ActivateBraking(DriveBrake driveBrake) {
        KServoAutoAction brakeRight = new KServoAutoAction(driveBrake.getBrakeRight(), BRAKE_RIGHT_POS);
        brakeRight.setName("brakeRight");
        this.addAction(brakeRight);

        KServoAutoAction brakeLeft = new KServoAutoAction(driveBrake.getBrakeLeft(), BRAKE_LEFT_POS);
        brakeLeft.setName("brakeLeft");
        this.addAction(brakeLeft);
    }

}
