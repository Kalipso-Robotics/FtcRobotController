package com.kalipsorobotics.localization;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.SharedData;

public class ResetOdometryToPosition extends Action {

    private final Position position;

    public ResetOdometryToPosition(Position position) {
        this.position = position;
    }

    @Override
    protected void update() {
        if (position == null) {
            KLog.e("ResetOdometryToPosition", "Position is null");
            return;
        }
        SharedData.setOdometryWheelPosition(position);
        SharedData.setOdometryIMUPosition(position);
    }
}
