
package com.kalipsorobotics.actions.revolverActions;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KLog;

public class RevolverTeleOp extends Action {

    boolean left;

    Revolver revolver;

    public RevolverTeleOp(Revolver revolver, boolean left){
        this.revolver = revolver;
        this.left = left;
    }

    @Override
    protected void update() {
//        if(revolver.getCurrentRevolverServoIndex() != 0 && revolver.getCurrentRevolverServoIndex() != 1 && revolver.getCurrentRevolverServoIndex() != 3) {
//            KLog.d("teleop", "not in position, " + revolver.getRevolverServo().getPosition());
//            revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);
//        }

        if (revolver.getCurrentRevolverServoIndex() == -1) {
            KLog.d("teleop", "not in position, " + revolver.getRevolverServo().getPosition());
            revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);
        }

        if (left) {
            KLog.d("teleop", "moving left");
            KLog.d("teleop", "current index: " + revolver.getCurrentRevolverServoIndex());
            if (revolver.getCurrentRevolverServoIndex() == 0) {
                revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_1);
            } else if (revolver.getCurrentRevolverServoIndex() == 1) {
                revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_2);
            } else if (revolver.getCurrentRevolverServoIndex() == 2) {
                revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);
            }
        } else if (!left) {
            if (revolver.getCurrentRevolverServoIndex() == 2) {
                revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_1);
            } else if (revolver.getCurrentRevolverServoIndex() == 1) {
                revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_0);
            } else if (revolver.getCurrentRevolverServoIndex() == 0) {
                revolver.getRevolverServo().setPosition(Revolver.REVOLVER_INDEX_2);
            }
        }

        isDone = true;
    }
}
