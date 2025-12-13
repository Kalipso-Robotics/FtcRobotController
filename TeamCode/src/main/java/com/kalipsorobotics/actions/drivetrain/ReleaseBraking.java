package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import com.kalipsorobotics.modules.DriveBrake;
import com.kalipsorobotics.utilities.KLog;

public class ReleaseBraking extends KActionSet {

    public static final double RELEASE_BRAKE_RIGHT_POS = 0.7095;
    public static final double RELEASE_BRAKE_LEFT_POS = 0.8199;

    public ReleaseBraking(DriveBrake driveBrake) {

        KServoAutoAction brakeRight = new KServoAutoAction(driveBrake.getBrakeRight(), RELEASE_BRAKE_RIGHT_POS);
        brakeRight.setName("brakeRight");
        this.addAction(brakeRight);

        KServoAutoAction brakeLeft = new KServoAutoAction(driveBrake.getBrakeLeft(), RELEASE_BRAKE_LEFT_POS);
        brakeLeft.setName("brakeLeft");
        this.addAction(brakeLeft);
        KLog.d("Braking", "Brakes released");
    }

}
