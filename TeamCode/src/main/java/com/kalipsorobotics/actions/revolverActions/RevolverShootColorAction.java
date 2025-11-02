package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.KActionSet;
import com.kalipsorobotics.actions.actionUtilities.WaitAction;
import com.kalipsorobotics.actions.shooter.PushBall;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Pusher;
import com.kalipsorobotics.modules.MotifColor;
import com.kalipsorobotics.modules.Revolver;

public class RevolverShootColorAction extends KActionSet {
    public RevolverShootColorAction(Revolver revolver, Pusher pusher, Intake intake, MotifColor shootColor, DetectColorsAction detectColorsAction) {

        RevolverMoveToColorAction revolverMoveToColorAction = new RevolverMoveToColorAction(revolver, shootColor, detectColorsAction);
        revolverMoveToColorAction.setName("revolverMoveToColorAction");
        this.addAction(revolverMoveToColorAction);

        WaitAction waitAction1 = new WaitAction(200);
        waitAction1.setName("waitAction1");
        this.addAction(waitAction1);
        waitAction1.setDependentActions(revolverMoveToColorAction);

        PushBall pushBall = new PushBall(pusher, intake);
        pushBall.setName("kickBall");
        pushBall.setDependentActions(revolverMoveToColorAction, waitAction1);
        this.addAction(pushBall);

        WaitAction waitAction2 = new WaitAction(1);
        waitAction2.setName("waitAction2");
        this.addAction(waitAction2);
        waitAction2.setDependentActions(pushBall);
    }
}
