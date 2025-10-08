
//package com.kalipsorobotics.actions.revolverActions;
//
//import com.kalipsorobotics.actions.actionUtilities.Action;
//import com.kalipsorobotics.modules.Revolver;
//
//public class RevolverTeleop extends Action {
//
//    Revolver revolver;
//
//    @Override
//    protected boolean checkDoneCondition() {
//        return false;
//    }
//
//    @Override
//    protected void update() {
//        if (gamepad1.left_stick_x > 0) {
//            revolver.setPosition(revolverPosition);
//            revolverPosition += 0.01;
//        } else if (gamepad1.left_stick_x < 0) {
//            revolver.setPosition(revolverPosition);
//            revolverPosition -= 0.01;
//        }
//    }
//}
