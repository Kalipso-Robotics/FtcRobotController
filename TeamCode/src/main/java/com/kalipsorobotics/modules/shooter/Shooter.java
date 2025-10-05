package com.kalipsorobotics.modules.shooter;

import android.util.Log;

import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.json.JSONException;
import java.io.IOException;

public class Shooter {
    private final OpModeUtilities opModeUtilities;
    private double rpm;
    double prevTicks = 0;
    double prevTimeMS = System.currentTimeMillis();

    private final DcMotor shooter1;


    private final DcMotor shooter2;


    private final KServo hood;

    private final KServo kickerRight;
    private final KServo kickerLeft;


    private final ShooterLutPredictor predictor;

    public KServo getKickerRight() {
        return kickerRight;
    }

    public KServo getKickerLeft() {
        return kickerLeft;
    }

    public Shooter(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        shooter1 = opModeUtilities.getHardwareMap().dcMotor.get("shooter1");
        shooter2 = opModeUtilities.getHardwareMap().dcMotor.get("shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo hood = opModeUtilities.getHardwareMap().servo.get("hood");
        if (hood == null) {
            opModeUtilities.getTelemetry().addData("Error", "Hood servo not found in hardware map");
        }
        this.hood = new KServo(hood, KServo.AXON_MAX_SPEED, 255, 0, false);

        Servo kickerRight = opModeUtilities.getHardwareMap().servo.get("kicker1");
        if (kickerRight == null) {
            opModeUtilities.getTelemetry().addData("Error", "Kicker1 servo not found in hardware map");
        }
        this.kickerRight = new KServo(kickerRight, KServo.AXON_MAX_SPEED, 255, 0, false);

        Servo kickerLeft = opModeUtilities.getHardwareMap().servo.get("kicker2");
        if (kickerLeft == null) {
            opModeUtilities.getTelemetry().addData("Error", "Kicker2 servo not found in hardware map");
        }
        this.kickerLeft = new KServo(kickerLeft, KServo.AXON_MAX_SPEED, 255, 0, false);



        ShooterLutPredictor temp = null;
        try {
            temp = new ShooterLutPredictor(opModeUtilities.getHardwareMap().appContext, "Shooter LUT.json");
        } catch (IOException | JSONException e) {
            opModeUtilities.getTelemetry().addData("Error", "Failed to load Shooter LUT: " + e.getMessage());
        }
        predictor = temp;
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

    /**
     * Calculate and set motor powers and hood position based on millimeter coordinates
     * @param xMM horizontal position in millimeters
     */
    private void setCalculatedPowerAndHood(double xMM) {
        if (predictor == null) {
            opModeUtilities.getTelemetry().addData("Warning", "Predictor not initialized");
            return;
        }

        // Convert millimeters to pixels (0.2608 pixels/mm)
        double xPixels = xMM * 0.2608;
        double yPixels = 1000 * 0.2608;

        ShooterLutPredictor.Prediction prediction = predictor.predict(xPixels, yPixels);

        Log.d("Hood", "Hood Pos: " + hood.getPosition());
        Log.d("Wheel", "Target RPS: " + prediction.rps + ", Current RPS: " + getRPS());

        // Note: This does NOT set motor power - use ShooterReady action for power ramping
        // This only sets the hood position
        hood.setPosition(prediction.hood);
    }

    public void stop() {
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    /**
     * Set motor power directly
     * @param power motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    /**
     * Get the target RPS for a given position and target
     * @param currentPosition current robot position from odometry (x, y, theta)
     * @param target target point to shoot at
     * @return target RPS for the shooter
     */
    public double getTargetRPS(Position currentPosition, Point target) {
        if (predictor == null) {
            opModeUtilities.getTelemetry().addData("Warning", "Predictor not initialized");
            return 0;
        }

        // Calculate distance between current position and target (ignoring theta)
        double dx = target.getX() - currentPosition.getX();
        double dy = target.getY() - currentPosition.getY();
        double distance = Math.sqrt((dx * dx) + (dy * dy));

        // Convert millimeters to pixels (0.2608 pixels/mm)
        double xPixels = distance * 0.2608;
        double yPixels = 1000 * 0.2608;

        ShooterLutPredictor.Prediction prediction = predictor.predict(xPixels, yPixels);
        return prediction.rps;
    }

    /**
     * Update hood position based on current position and target
     * @param currentPosition current robot position from odometry (x, y, theta)
     * @param target target point to shoot at
     */
    public void updateHoodFromPosition(Position currentPosition, Point target) {
        // Calculate distance between current position and target (ignoring theta)
        double dx = target.getX() - currentPosition.getX();
        double dy = target.getY() - currentPosition.getY();
        double distance = Math.sqrt((dx * dx) + (dy * dy));

        // Use distance as xMM to set hood position
        setCalculatedPowerAndHood(distance);
    }



}
