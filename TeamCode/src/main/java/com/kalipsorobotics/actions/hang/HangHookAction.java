package com.kalipsorobotics.actions.hang;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAutoAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.robotcore.hardware.Servo;

public class HangHookAction extends Action {

    Servo hang1, hang2;

    public HangHookAction(Outtake outtake) {
        this.hang1 = outtake.getHangHook1();
        this.dependentAction = new DoneStateAction();
        hang1.setPosition(0.4);
    }

    @Override
    public boolean checkDoneCondition() {
        if (hasStarted) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if (!hasStarted) {
            hang1.setPosition(0.00);
            Log.d("hang", "set position");
            hasStarted = true;
        }
    }
}