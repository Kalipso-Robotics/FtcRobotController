package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KGamePad;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SetAutoDelayAction extends Action {

    OpModeUtilities opModeUtilities;
    KGamePad kGamePad1;
    int delayMS = 0;

    /**
     * Remember to set dependency for first action after wait.
     *
     * <pre>
     * SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
     * setAutoDelayAction.setName("setAutoDelayAction");
     *
     * while(!setAutoDelayAction.getIsDone() && opModeInInit()) {
     * setAutoDelayAction.updateCheckDone();
     * }
     *
     * WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMS());
     * delayBeforeStart.setName("delayBeforeStart");
     * redAutoNear.addAction(delayBeforeStart);
     * </pre>
     *
     * @param opModeUtilities The utility object providing access to OpMode functions.
     * @param gamepad         The gamepad object used to configure the delay time.
     */

    public SetAutoDelayAction(OpModeUtilities opModeUtilities, Gamepad gamepad) {
        kGamePad1 = new KGamePad(gamepad);
        this.opModeUtilities = opModeUtilities;
    }

    public int getTimeMS() {
        return delayMS;
    }

    @Override
    protected boolean isUpdateDone() {
        if (isDone) {
            return true;
        }

        if (kGamePad1.isLeftBumperFirstPressed() && kGamePad1.isRightBumperFirstPressed()){
            opModeUtilities.getTelemetry().addLine("CONFIRMED DELAY");
            opModeUtilities.getTelemetry().addData("DELAY TIME IN MS ", getTimeMS());
            opModeUtilities.getTelemetry().update();
            isDone = true;
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void update() {
        if (kGamePad1.isDpadLeftFirstPressed()) {
            delayMS -= 500;
        } else if (kGamePad1.isDpadRightFirstPressed()) {
            delayMS += 500;
        }

        if (delayMS < 0) {
            delayMS = 0;
        }
        opModeUtilities.getTelemetry().addLine("DPAD LEFT DECREMENT \n" +
                " DPAD RIGHT INCREMENT \n" +
                " LEFT AND RIGHT BUMPER CONFIRM DELAY \n");
        opModeUtilities.getTelemetry().addLine("INIT FINISHED");
        opModeUtilities.getTelemetry().addData("DELAY TIME IN MS ", getTimeMS());
        opModeUtilities.getTelemetry().update();
    }
}
