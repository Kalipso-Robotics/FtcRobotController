package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.hang.HangHookAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Outtake;

public class AutoHangAction extends Action {
    MoveOuttakeLSAction moveLSUp;
    WaitAction waitAction;
    MoveOuttakeLSAction moveLSDown;
    WaitAction waitActionTwo;
    MoveOuttakeLSAction pullUp;
    HangHookAction hangHookAction;

    public AutoHangAction(Outtake outtake) {
        this.dependentActions.add(new DoneStateAction());

        moveLSUp = new MoveOuttakeLSAction(outtake, CalculateTickPer.inchToTicksLS(28));

        waitAction = new WaitAction(400);
        waitAction.setDependentActions(moveLSUp);

        moveLSDown = new MoveOuttakeLSAction(outtake, CalculateTickPer.inchToTicksLS(-5));
        moveLSDown.setDependentActions(waitAction);

        waitActionTwo = new WaitAction(300);
        waitActionTwo.setDependentActions(moveLSDown);

        pullUp = new MoveOuttakeLSAction(outtake, CalculateTickPer.inchToTicksLS(-20));
        pullUp.setDependentActions(waitActionTwo);

        hangHookAction = new HangHookAction(outtake);
        hangHookAction.setDependentActions(waitActionTwo);
    }

    @Override
    public boolean checkDoneCondition() {
        return false;
    }

    @Override
    public void update() {
        moveLSUp.updateCheckDone();
        waitAction.updateCheckDone();
        moveLSDown.updateCheckDone();
        waitActionTwo.updateCheckDone();
        pullUp.updateCheckDone();
        hangHookAction.updateCheckDone();
    }
}
