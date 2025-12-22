package com.kalipsorobotics.localization;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToPosition extends Action {

    private final Position position;

    public ResetOdometryToPosition(Position position) {
        this.position = position;
    }

    @Override
    protected void update() {
        SharedData.setOdometryWheelPosition(position);
        SharedData.setOdometryIMUPosition(position);
    }
}
