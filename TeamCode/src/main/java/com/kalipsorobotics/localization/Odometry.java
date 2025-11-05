package com.kalipsorobotics.localization;

import android.os.SystemClock;

import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.DriveTrain;

import java.util.HashMap;


public class Odometry {
    final static private double TRACK_WIDTH_MM = 297;
    //maybe double check BACK Distance
    static private final double BACK_DISTANCE_TO_MID_ROBOT_MM = -70;
    private static Odometry single_instance = null;
    final private PositionHistory wheelPositionHistory = new PositionHistory();
    final private PositionHistory wheelIMUPositionHistory = new PositionHistory();
    OpModeUtilities opModeUtilities;
    HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionHistoryHashMap = new HashMap<>();
    IMUModule imuModule;
    //200-182 offset compare to line between parallel odo pods
    //negative if robot center behind parallel wheels
    //final private static double ROBOT_CENTER_OFFSET_MM = -18;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;

    private final double rightOffset;
    private final double leftOffset;
    private final double backOffset;
    private volatile double prevRightDistanceMM;
    private volatile double prevLeftDistanceMM;
    volatile private double prevBackDistanceMM;
    private volatile long prevTime;
    private volatile double currentImuHeading;
    private volatile double prevImuHeading;
//    private final double MM_TO_INCH = 1/25.4;

    private Odometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule,
                     Position startPosMMRAD) {
        this.opModeUtilities = opModeUtilities;
        resetHardware(driveTrain, imuModule, this);
        this.rightOffset = this.getRightEncoderMM();
        this.leftOffset = this.getLeftEncoderMM();
        this.backOffset = this.getBackEncoderMM();

        this.wheelPositionHistory.setCurrentPosition(startPosMMRAD);
        this.wheelIMUPositionHistory.setCurrentPosition(startPosMMRAD);

        prevTime = SystemClock.elapsedRealtime();
        prevImuHeading = getIMUHeading();
        currentImuHeading = prevImuHeading;
        prevRightDistanceMM = getRightEncoderMM();
        prevLeftDistanceMM = getLeftEncoderMM();
        prevBackDistanceMM = getBackEncoderMM();
    }

    private Odometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule,
                     double startX, double startY, double startThetaDeg) {
        this(opModeUtilities, driveTrain, imuModule, new Position(startX, startY, Math.toRadians(startThetaDeg)));
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        } else {
            resetHardware(driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, GoBildaOdoModule goBildaOdoModule, Position startPosMMRAD) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, startPosMMRAD);
        } else {
            resetHardware(driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, GoBildaOdoModule goBildaOdoModule, double startX, double startY, double startThetaDeg) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, startX, startY, Math.toRadians(startThetaDeg));
        } else {
            resetHardware(driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    private static void resetHardware(DriveTrain driveTrain, IMUModule imuModule, Odometry odometry) {
        SharedData.resetOdometryPosition();
        odometry.imuModule = imuModule;
        odometry.rightEncoder = driveTrain.getRightEncoder();
        odometry.leftEncoder = driveTrain.getLeftEncoder();
        odometry.backEncoder = driveTrain.getBackEncoder();
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static double ticksToMM(double ticks) {
        final double DEAD_WHEEL_RADIUS_MM = 24;
        final double TICKS_PER_REV = 2000;
        final double TICKS_TO_MM = 2.0 * Math.PI * DEAD_WHEEL_RADIUS_MM / TICKS_PER_REV;

        return ticks * TICKS_TO_MM;
    }

    public double getRightEncoderMM() {
        //corresponds to fRight
        //direction FORWARD
        //negative because encoder directions
        return ticksToMM(rightEncoder.getCurrentPosition()) - rightOffset;
        //return ticksToMM(rightEncoder.getCurrentPosition());
    }
    public double getLeftEncoderMM() {
        //corresponds to fLeft
        //direction FORWARD
        //positive because encoder directions
        return ticksToMM(leftEncoder.getCurrentPosition()) - leftOffset;
    }
    public double getBackEncoderMM() {
        //corresponds to bRight
        //direction REVERSE
        //positive because encoder directions
        return ticksToMM(backEncoder.getCurrentPosition()) - backOffset;
        //return ticksToMM(backEncoder.getCurrentPosition());
    }

    private Velocity calculateRelativeDeltaWheel(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;
        double deltaTheta = MathFunctions.angleWrapRad((deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM);



        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * deltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, deltaTheta);

        return velocity;
    }

    private Velocity calculateRelativeDeltaWheelIMU(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double imuDeltaTheta = MathFunctions.angleWrapRad(currentImuHeading - prevImuHeading);

//
//        double rawImuDeltaTheta = MathFunctions.angleWrapRad(currentImuHeading - prevImuHeading);
//        double wheelDeltaTheta  = (deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM;
//
//        //this limit should match your loop time; 20°/cycle is safe for FTC.
//        double maxTurnPerStep = Math.toRadians(10);
//
//        if (Math.abs(rawImuDeltaTheta) > maxTurnPerStep) {
//            //this is likely an IMU shock / 180° wrap → fall back to wheel
//            imuDeltaTheta = wheelDeltaTheta;
//        } else {
//            //IMU is usually better for heading, but we still keep 30% wheel
//            // to improve continuity with the dead-wheel translation.
//            imuDeltaTheta = rawImuDeltaTheta;
//        }

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * imuDeltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, imuDeltaTheta);

        return velocity;
    }

    private Velocity linearToArcDelta(Velocity relativeDelta) {
        if (Math.abs(relativeDelta.getTheta()) < 1e-4) {
            return relativeDelta;
        }

        //KLog.d("odometry", "linearDelta " + relativeDelta);
        double forwardRadius = relativeDelta.getX() / relativeDelta.getTheta();
        double strafeRadius = relativeDelta.getY() / relativeDelta.getTheta();

        double relDeltaX =
                forwardRadius * Math.sin(relativeDelta.getTheta()) + -strafeRadius * (1 - Math.cos(relativeDelta.getTheta()));

        double relDeltaY =
                +strafeRadius * Math.sin(relativeDelta.getTheta()) + forwardRadius * (1 - Math.cos(relativeDelta.getTheta()));

        double relDeltaTheta =
                relativeDelta.getTheta();

        Velocity arcDelta = new Velocity(relDeltaX, relDeltaY, relDeltaTheta);
        if (Math.abs(arcDelta.getTheta()) > 0.01) {
            //KLog.d("odometry new arc delta", "arcDelta4" + arcDelta);
        }
        return arcDelta;
    }

    //converts to global
    private Velocity rotate(Velocity relativeDelta, Position previousGlobalPosition) {
        double sinTheta = Math.sin(previousGlobalPosition.getTheta());
        double cosTheta = Math.cos(previousGlobalPosition.getTheta());

        double deltaX = relativeDelta.getX();
        double deltaY = relativeDelta.getY();

        double newX = deltaX * cosTheta - deltaY * sinTheta;
        double newY = deltaY * cosTheta + deltaX * sinTheta;

        double newTheta = relativeDelta.getTheta();

        return new Velocity(newX, newY, newTheta);
    }

    private Position calculateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);
        //KLog.d("global delta", globalDelta.toString());
        Position position = previousGlobalPosition.add(globalDelta);
        return position;
    }

    private void updateWheelPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double timeElapsedSeconds) {
        Velocity wheelRelDelta = calculateRelativeDeltaWheel(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedSeconds * 1000);
        wheelRelDelta = linearToArcDelta(wheelRelDelta);
        Position globalPosition = calculateGlobal(wheelRelDelta, wheelPositionHistory.getCurrentPosition());
        wheelPositionHistory.setCurrentPosition(globalPosition);
        wheelPositionHistory.setCurrentVelocity(wheelRelDelta, timeElapsedSeconds * 1000);
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL, wheelPositionHistory);
    }

    private void updateWheelIMUPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                   double timeElapsedSeconds) {
        Velocity wheelIMURelDelta = calculateRelativeDeltaWheelIMU(rightDistanceMM, leftDistanceMM, backDistanceMM,
                timeElapsedSeconds * 1000);
        wheelIMURelDelta = linearToArcDelta(wheelIMURelDelta);
        Position globalPosition = calculateGlobal(wheelIMURelDelta, wheelIMUPositionHistory.getCurrentPosition());
        wheelIMUPositionHistory.setCurrentPosition(globalPosition);
        wheelIMUPositionHistory.setCurrentVelocity(wheelIMURelDelta, timeElapsedSeconds * 1000); //mm/ms
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL_IMU, wheelIMUPositionHistory);

    }

    public HashMap<OdometrySensorCombinations, PositionHistory> updateAll() {
        KLog.d("updatepos", "updatepos");
        double rightDistanceMM = getRightEncoderMM();
        double leftDistanceMM = getLeftEncoderMM();
        double backDistanceMM = getBackEncoderMM();
        currentImuHeading = getIMUHeading();

        KLog.d("IMU_Heading", "Heading " + Math.toDegrees(currentImuHeading));
        KLog.d("IMU_Prev_Heading", "PrevHeading" + Math.toDegrees(prevImuHeading));

        //KLog.d("updatepos", rightDistanceMM + " " + leftDistanceMM + " " + backDistanceMM);

        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsedSeconds = (currentTime - prevTime) / 1000.0;

        updateWheelPos(rightDistanceMM,leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateWheelIMUPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);

        //KLog.d("currentpos", "current pos " + currentPosition.toString());
        prevTime = currentTime;

        prevRightDistanceMM = rightDistanceMM;
        prevLeftDistanceMM = leftDistanceMM;
        prevBackDistanceMM = backDistanceMM;

        prevImuHeading = currentImuHeading;
        SharedData.setOdometryPosition(odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU).getCurrentPosition());
        SharedData.setOdometryPositionMap(odometryPositionHistoryHashMap);
        KLog.d("updatepos", "odometry pos " + SharedData.getOdometryPosition());
        return odometryPositionHistoryHashMap;
    }

    public Position update() {
        //IMU
        HashMap<OdometrySensorCombinations, PositionHistory> positionHistoryHashMap = updateAll();
        PositionHistory positionHistory = positionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU);
        if (positionHistory == null) {
            throw new RuntimeException("WHEEL_IMU Position History Null");
        }
        return  positionHistory.getCurrentPosition();
    }

    private PositionHistory getCurrentPositionHistory() {
        return wheelIMUPositionHistory;
    }

    public double getIMUHeading() {
        return -Math.toRadians(imuModule.getIMU().getRobotYawPitchRollAngles().getYaw());
    }
}