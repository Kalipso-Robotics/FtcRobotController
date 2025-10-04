package com.kalipsorobotics.modules;

import com.kalipsorobotics.actions.actionUtilities.Action;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends Action {
    private final OpModeUtilities opModeUtilities;
    private double rpm;
    double prevTicks = 0;
    double prevTimeMS = System.currentTimeMillis();

    private final DcMotor shooter1;


    private final DcMotor shooter2;


    private final KServo hood;

    public Shooter(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        shooter1 = opModeUtilities.getHardwareMap().dcMotor.get("shooter1");
        shooter2 = opModeUtilities.getHardwareMap().dcMotor.get("shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo hoodServo = opModeUtilities.getHardwareMap().servo.get("hood");
        hood = new KServo(hoodServo, KServo.AXON_MAX_SPEED, 255, 0, false);
    }

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

    public double getHoodPosition() {
        return hood.getPosition();
    }

    public double getRPS() {
        double currentTicks = shooter1.getCurrentPosition();
        double currentTimeMS = System.currentTimeMillis();
        double deltaTicks = Math.abs(currentTicks - prevTicks);
        double deltaRotation = CalculateTickPer.ticksToRotation6000RPM(deltaTicks);
        double deltaTimeMS = currentTimeMS - prevTimeMS;
        double rps = (deltaRotation) / (deltaTimeMS / 1000);
        prevTicks = currentTicks;
        prevTimeMS = currentTimeMS;
        return (rps);
    }


    public KServo getHood() {
        return hood;
    }


    public DcMotor getShooter1() {
        return shooter1;
    }


    public DcMotor getShooter2() {
        return shooter2;
    }

}
