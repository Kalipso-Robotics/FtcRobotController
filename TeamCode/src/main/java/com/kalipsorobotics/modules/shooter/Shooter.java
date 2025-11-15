package com.kalipsorobotics.modules.shooter;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.utilities.KMotor;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    public static final double MM_TO_PIXEL_RATIO = 0.2608;

    public static final double GOAL_HEIGHT_MM = 750.0;//838
    public static final double GOAL_HEIGHT_PIXELS = GOAL_HEIGHT_MM * MM_TO_PIXEL_RATIO;

    public static final Point RED_TARGET_FROM_FAR = new Point(((144-6) * 25.4) - 200, ((72-6) * 25.4) - 190 - (196.85));//red

    public static final double HOOD_OFFSET = 0.25;

    public final double TARGET_RPS_TOLERANCE = 1;


    private final OpModeUtilities opModeUtilities;

    private final KMotor shooter1;
    private final KMotor shooter2;


    private final KServo hood;

//    private final KCRServo pusherRight;
//    private final KCRServo pusherLeft;
//

    private final IShooterPredictor predictor;

//    public KCRServo getPusherRight() {
//        return pusherRight;
//    }
//
//    public KCRServo getPusherLeft() {
//        return pusherLeft;
//    }

    public Shooter(OpModeUtilities opModeUtilities) {

        this.opModeUtilities = opModeUtilities;
        DcMotor motor1 = opModeUtilities.getHardwareMap().dcMotor.get("shooter1");
        DcMotor motor2 = opModeUtilities.getHardwareMap().dcMotor.get("shooter2");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        shooter1 = new KMotor(motor1, 1.0/100.0, 0, 0);
        shooter2 = new KMotor(motor2, 1.0/100.0, 0, 0);
        Servo hood = opModeUtilities.getHardwareMap().servo.get("hood");
        if (hood == null) {
            opModeUtilities.getTelemetry().addData("Error", "Hood servo not found in hardware map");
        }
        this.hood = new KServo(hood, KServo.AXON_MAX_SPEED, 255, 0, false);

//        CRServo kickerRight = opModeUtilities.getHardwareMap().crservo.get("pusherRight");
//        if (kickerRight == null) {
//            opModeUtilities.getTelemetry().addData("Error", "Kicker1 servo not found in hardware map");
//        }
//        this.pusherRight = new KCRServo(kickerRight, false);
//
//        CRServo kickerLeft = opModeUtilities.getHardwareMap().crservo.get("pusherLeft");
//        if (kickerLeft == null) {
//            opModeUtilities.getTelemetry().addData("Error", "Kicker2 servo not found in hardware map");
//        }
//        this.pusherLeft = new KCRServo(kickerLeft, true);



        // Use manual tuned data lookup (no file loading needed)
        // To swap implementations, change this line:
        predictor = new ShooterInterpolationDataLookup();
        // Alternative: predictor = new ShooterLutPredictorAdapter(opModeUtilities.getHardwareMap().appContext, "CappedShooterLUT.bin");
    }

    public double getHoodPosition() {
        return hood.getPosition();
    }

    public double getRPS() {
        return shooter1.getRPS();
    }

    public boolean isAtTargetRPS() {
        boolean isWithinTarget = shooter1.isAtTargetRPS(TARGET_RPS_TOLERANCE);
        return isWithinTarget;
    }


    public KServo getHood() {
        return hood;
    }


    public KMotor getShooter1() {
        return shooter1;
    }


    public KMotor getShooter2() {
        return shooter2;
    }

    /**
     * Calculate shooter parameters based on distance to target
     * @param distanceMM distance to target in millimeters
     * @return shooter parameters containing RPS and hood position
     */
    public IShooterPredictor.ShooterParams getPrediction(double distanceMM) {
        return predictor.predict(distanceMM);
    }

    /**
     * Set hood position based on distance to target
     * @param distanceMM distance to target in millimeters
     */
    private void setCalculatedHood(double distanceMM) {
        IShooterPredictor.ShooterParams params = getPrediction(distanceMM);

        KLog.d("Hood", "Hood Pos: " + hood.getPosition());
        KLog.d("Wheel", "Target RPS: " + params.rps + ", Current RPS: " + getRPS());

        // Note: This does NOT set motor power - use ShooterReady action for power ramping
        // This only sets the hood position
        // Hood offset is already included in predictor implementations
        KLog.d("Hood", "Hood Pos: " + params.hoodPosition);
        hood.setPosition(params.hoodPosition);
    }

    public void stop() {
        shooter1.stop();
        shooter2.stop();
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
     * Set target RPS for both motors using PID control
     * @param targetRPS desired rotations per second
     */
    public void goToRPS(double targetRPS) {
        shooter1.goToRPS(targetRPS);
        shooter2.goToRPS(targetRPS);
    }

    /**
     * Get the target RPS for a given position and target
     * @param currentPosition current robot position from odometry (x, y, theta)
     * @param target target point to shoot at
     * @return target RPS for the shooter
     */
    public double getDistance(Position currentPosition, Point target) {
        double dx = target.getX() - currentPosition.getX();
        double dy = target.getY() - currentPosition.getY();
        double distance = Math.sqrt((dx * dx) + (dy * dy));
        return distance;
    }
    public double getTargetRPS(Position currentPosition, Point target) {
        // Calculate distance between current position and target (ignoring theta)
        double distance = getDistance(currentPosition, target);

        IShooterPredictor.ShooterParams params = getPrediction(distance);

        KLog.d("TargetRPS", "Current: (" + currentPosition.getX() + ", " + currentPosition.getY() +
              "), Target: (" + target.getX() + ", " + target.getY() +
              "), Distance: " + distance + "mm, Predicted RPS: " + params.rps + ", Hood: " + params.hoodPosition);

        return params.rps;
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

        KLog.d("Distance", "distance: " + distance);
        // Use distance as xMM to set hood position
        setCalculatedHood(distance);
    }

    public static double mmToPixel(double mm) {
        return mm * MM_TO_PIXEL_RATIO;
    }

    public static double pixelToMM(double pixel) {
        return pixel / MM_TO_PIXEL_RATIO;
    }



}
