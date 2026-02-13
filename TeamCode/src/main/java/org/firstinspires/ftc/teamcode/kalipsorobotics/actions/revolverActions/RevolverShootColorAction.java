package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.revolverActions;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.MotifColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Revolver;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;

public class RevolverShootColorAction extends KActionSet {
    public RevolverShootColorAction(Shooter shooter, Revolver revolver, Stopper stopper, Intake intake, MotifColor shootColor, DetectColorsAction detectColorsAction) {

        RevolverMoveToColorAction revolverMoveToColorAction = new RevolverMoveToColorAction(revolver, shootColor, detectColorsAction);
        revolverMoveToColorAction.setName("revolverMoveToColorAction");
        this.addAction(revolverMoveToColorAction);

        WaitAction waitAction1 = new WaitAction(200);
        waitAction1.setName("waitAction1");
        this.addAction(waitAction1);
        waitAction1.setDependentActions(revolverMoveToColorAction);

        PushBall pushBall = new PushBall(stopper, intake);
        pushBall.setName("kickBall");
        pushBall.setDependentActions(revolverMoveToColorAction, waitAction1);
        this.addAction(pushBall);

        WaitAction waitAction2 = new WaitAction(1);
        waitAction2.setName("waitAction2");
        this.addAction(waitAction2);
        waitAction2.setDependentActions(pushBall);
    }
}
