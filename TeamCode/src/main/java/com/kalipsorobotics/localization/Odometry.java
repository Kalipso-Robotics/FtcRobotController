package com.kalipsorobotics.localization;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.DriveTrain;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;


public class Odometry {
    final static private double TRACK_WIDTH_MM = 297;
    //maybe double check BACK Distance
    static private final double BACK_DISTANCE_TO_MID_ROBOT_MM = -70;
    private static Odometry single_instance = null;
    final private PositionHistory wheelIMUPositionHistory = new PositionHistory();
    final private PositionHistory gobildaPositionHistory = new PositionHistory();
    OpModeUtilities opModeUtilities;
    HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionHistoryHashMap = new HashMap<>();
    IMUModule imuModule;
    GoBildaOdoModule goBildaOdoModule;
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

    private Odometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, GoBildaOdoModule goBildaOdoModule,
                     Position startPosMMRAD) {
        this.opModeUtilities = opModeUtilities;
        resetHardware(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, this);
        goBildaOdoModule.getGoBildaPinpointDriver().update();
        this.rightOffset = ticksToMM(goBildaOdoModule.getGoBildaPinpointDriver().getEncoderX());
        this.leftOffset = this.getLeftEncoderMM();
        this.backOffset = ticksToMM(goBildaOdoModule.getGoBildaPinpointDriver().getEncoderY());

        this.wheelIMUPositionHistory.setCurrentPosition(startPosMMRAD);
        this.gobildaPositionHistory.setCurrentPosition(startPosMMRAD);
        ////Log.d("purepursaction_debug_odo_wheel", "init jimmeh" + currentPosition.toString());
        prevTime = SystemClock.elapsedRealtime();
        prevImuHeading = getIMUHeading();
        currentImuHeading = prevImuHeading;
        prevRightDistanceMM = getRightEncoderMM();
        prevLeftDistanceMM = getLeftEncoderMM();
        prevBackDistanceMM = getBackEncoderMM();
    }

    private Odometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, GoBildaOdoModule goBildaOdoModule,
                     double startX, double startY, double startThetaDeg) {
        this(opModeUtilities, driveTrain, imuModule,  goBildaOdoModule, new Position(startX, startY, Math.toRadians(startThetaDeg)));
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, GoBildaOdoModule goBildaOdoModule) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, 0, 0, 0);
        } else {
            resetHardware(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, GoBildaOdoModule goBildaOdoModule, Position startPosMMRAD) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, startPosMMRAD);
        } else {
            resetHardware(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, GoBildaOdoModule goBildaOdoModule, double startX, double startY, double startThetaDeg) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, startX, startY, Math.toRadians(startThetaDeg));
        } else {
            resetHardware(opModeUtilities, driveTrain, imuModule, goBildaOdoModule, single_instance);
        }
        return single_instance;
    }

    private static void resetHardware(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, GoBildaOdoModule goBildaOdoModule, Odometry odometry) {
        odometry.imuModule = imuModule;
        odometry.goBildaOdoModule = goBildaOdoModule;
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
        return ticksToMM(goBildaOdoModule.getGoBildaPinpointDriver().getEncoderX()) - rightOffset;
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
        return -(ticksToMM(goBildaOdoModule.getGoBildaPinpointDriver().getEncoderY()) - backOffset);
        //return ticksToMM(backEncoder.getCurrentPosition());
    }



    private Velocity calculateRelativeDeltaWheelIMU(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double imuDeltaTheta = currentImuHeading - prevImuHeading;

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * imuDeltaTheta);

        Velocity velocity = new Velocity(deltaX, deltaY, imuDeltaTheta);

        return velocity;
    }

    private Velocity linearToArcDelta(Velocity relativeDelta) {
        if (Math.abs(relativeDelta.getTheta()) < 1e-4) {
            return relativeDelta;
        }

        //Log.d("odometry", "linearDelta " + relativeDelta);
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
            //Log.d("odometry new arc delta", "arcDelta4" + arcDelta);
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
        //Log.d("global delta", globalDelta.toString());
        Position position = previousGlobalPosition.add(globalDelta);
        return position;
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
    private void updateGobilda(double timeElapsedSeconds) {
        goBildaOdoModule.getGoBildaPinpointDriver().update();
        Pose2D position = (goBildaOdoModule.getGoBildaPinpointDriver().getPosition());
        //Position newPosition = new Position(-position.getX(DistanceUnit.MM), position.getY(DistanceUnit.MM), position.getHeading(AngleUnit.RADIANS));
        gobildaPositionHistory.setCurrentPosition(Position.pose2DtoPosition(position));
        gobildaPositionHistory.setCurrentVelocity(Velocity.pose2DtoVelocity(goBildaOdoModule.getGoBildaPinpointDriver().getVelocity()), timeElapsedSeconds);
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.GOBILDA, gobildaPositionHistory);
    }
    public HashMap<OdometrySensorCombinations, PositionHistory> updatePositionAll() {
        Log.d("updatepos", "updatepos");
        double rightDistanceMM = getRightEncoderMM();
        double leftDistanceMM = getLeftEncoderMM();
        double backDistanceMM = getBackEncoderMM();
        currentImuHeading = getIMUHeading();

        Log.d("IMU_Heading", "Heading " + Math.toDegrees(currentImuHeading));
        Log.d("IMU_Prev_Heading", "PrevHeading" + Math.toDegrees(prevImuHeading));

        //Log.d("updatepos", rightDistanceMM + " " + leftDistanceMM + " " + backDistanceMM);

        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsedSeconds = (currentTime - prevTime) / 1000.0;


        updateWheelIMUPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateGobilda(timeElapsedSeconds);

        //Log.d("currentpos", "current pos " + currentPosition.toString());
        prevTime = currentTime;

        prevRightDistanceMM = rightDistanceMM;
        prevLeftDistanceMM = leftDistanceMM;
        prevBackDistanceMM = backDistanceMM;

        prevImuHeading = currentImuHeading;
        SharedData.setOdometryPosition(odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU).getCurrentPosition());
        SharedData.setOdometryPositionMap(odometryPositionHistoryHashMap);
        Log.d("updatepos", "updatepos done");
        return odometryPositionHistoryHashMap;
    }

    public Position update() {
        //IMU
        HashMap<OdometrySensorCombinations, PositionHistory> positionHistoryHashMap = updatePositionAll();
        PositionHistory positionHistory = positionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU);
        if (positionHistory == null) {
            throw new RuntimeException("WHEEL_IMU Position History Null");
        }
        return  positionHistory.getCurrentPosition();
    }

    public PositionHistory getCurrentPositionHistory() {
        return wheelIMUPositionHistory;
    }

    public double getIMUHeading() {
        return -Math.toRadians(imuModule.getIMU().getRobotYawPitchRollAngles().getYaw());
    }

}