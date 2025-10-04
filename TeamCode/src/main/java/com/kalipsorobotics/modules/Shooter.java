package com.kalipsorobotics.modules;

import com.kalipsorobotics.actions.actionUtilities.Action;

public class Shooter extends Action {

    @Override
    protected boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }



    }



}
