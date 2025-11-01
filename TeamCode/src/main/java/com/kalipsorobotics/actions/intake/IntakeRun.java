package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeRun extends Action {

    private Intake intake;
//    private int lastPos;
//    private long lastTime;


    public IntakeRun(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void update() {
        if (isDone) {
            return;
        }

        intake.getIntakeMotor().setPower(1);



//        lastPos = intake.getIntakeMotor().getCurrentPosition();
//        lastTime = System.currentTimeMillis();
    }

    // checks current to see if motor has stalled
}
