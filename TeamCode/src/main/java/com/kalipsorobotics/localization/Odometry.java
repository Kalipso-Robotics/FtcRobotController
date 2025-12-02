package com.kalipsorobotics.localization;

import android.os.SystemClock;

import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Velocity;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.modules.DriveTrain;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.HashMap;
import java.util.List;


public class Odometry {
    private boolean isOdometryUnhealthy = false;
    private int unhealthyCounter = 0;
    private boolean shouldFallbackToWheelTheta = true;
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
    private List<LynxModule> allHubs;

    private final double rightOffset;
    private final double leftOffset;
    private final double backOffset;
    private double prevRightDistanceMM;
    private double prevLeftDistanceMM;
    volatile private double prevBackDistanceMM;
    private long prevTime;
    private double currentImuHeading;
    private double prevImuHeading;
//    private final double MM_TO_INCH = 1/25.4;

    private Odometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule,
                     Position startPosMMRad) {
        KLog.d("Odometry_debug_OpMode_Transfer", "New Instance");
        this.opModeUtilities = opModeUtilities;
        resetHardware(opModeUtilities, driveTrain, imuModule, this);
        this.rightOffset = this.getRightEncoderMM();
        this.leftOffset = this.getLeftEncoderMM();
        this.backOffset = this.getBackEncoderMM();

        this.wheelPositionHistory.setCurrentPosition(startPosMMRad);
        this.wheelIMUPositionHistory.setCurrentPosition(startPosMMRad);

        prevTime = SystemClock.elapsedRealtime();
        prevImuHeading = getIMUHeading();
        currentImuHeading = prevImuHeading;
        prevRightDistanceMM = getRightEncoderMM();
        prevLeftDistanceMM = getLeftEncoderMM();
        prevBackDistanceMM = getBackEncoderMM();
    }

    private Odometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule,
                     double startXMM, double startYMM, double startThetaRad) {
        this(opModeUtilities, driveTrain, imuModule, new Position(startXMM, startYMM, startThetaRad));
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        } else {
            KLog.d("Odometry_debug_OpMode_Transfer", "Reuse Instance" + single_instance.toString());
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, Position startPosMMRad) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, startPosMMRad);
        } else {
            KLog.d("Odometry_debug_OpMode_Transfer", "Reuse Instance" + single_instance.toString());
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, double startXMM, double startYMM, double startThetaRad) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, startXMM, startYMM, startThetaRad);
        } else {
            KLog.d("Odometry_debug_OpMode_Transfer", "Reuse Instance" + single_instance.toString());
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    private static void resetHardware(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule, Odometry odometry) {
        odometry.opModeUtilities = opModeUtilities;
        odometry.imuModule = imuModule;
        odometry.rightEncoder = driveTrain.getRightEncoder();
        odometry.leftEncoder = driveTrain.getLeftEncoder();
        odometry.backEncoder = driveTrain.getBackEncoder();

        // Initialize bulk read for improved encoder reading performance
        odometry.allHubs = opModeUtilities.getHardwareMap().getAll(LynxModule.class);
        for (LynxModule module : odometry.allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        KLog.d("Odometry_BulkRead", "Initialized MANUAL bulk caching for " + odometry.allHubs.size() + " hub(s)");
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
        try {
            int position = rightEncoder.getCurrentPosition();
            KLog.d("Odometry_Encoder", "right encoder" + position);
            return ticksToMM(position) - rightOffset;
        } catch (Exception e) {
            KLog.d("Odometry_Encoder_Error", "Right encoder read failed: " + e.getMessage());
            // Return last known value to avoid position jumps
            return prevRightDistanceMM;
        }
    }

    public double getLeftEncoderMM() {
        //corresponds to fLeft
        //direction FORWARD
        //positive because encoder directions
        try {
            int position = leftEncoder.getCurrentPosition();
            KLog.d("Odometry_Encoder", "left encoder" + position);
            return ticksToMM(position) - leftOffset;
        } catch (Exception e) {
            KLog.d("Odometry_Encoder_Error", "Left encoder read failed: " + e.getMessage());
            // Return last known value to avoid position jumps
            return prevLeftDistanceMM;
        }
    }

    public double getBackEncoderMM() {
        //corresponds to bRight
        //direction REVERSE
        //positive because encoder directions
        try {
            int position = backEncoder.getCurrentPosition();
            KLog.d("Odometry_Encoder", "back encoder" + position);
            return ticksToMM(position) - backOffset;
        } catch (Exception e) {
            KLog.d("Odometry_Encoder_Error", "Back encoder read failed: " + e.getMessage());
            // Return last known value to avoid position jumps
            return prevBackDistanceMM;
        }
    }

    public boolean allEncoderZero() {
        return (rightEncoder.getCurrentPosition() == 0) && (leftEncoder.getCurrentPosition() == 0) && (backEncoder.getCurrentPosition() == 0);
    }

    public boolean anyEncoderZero() {
        return (rightEncoder.getCurrentPosition() == 0) || (leftEncoder.getCurrentPosition() == 0) || (backEncoder.getCurrentPosition() == 0);
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

        KLog.d("Odometry_Delta_Theta", "Raw IMU Current Heading" + currentImuHeading + " Prev IMU Heading " + prevImuHeading);
        double imuDeltaTheta = MathFunctions.angleWrapRad(currentImuHeading - prevImuHeading);



        double rawImuDeltaTheta = MathFunctions.angleWrapRad(currentImuHeading - prevImuHeading);
        double wheelDeltaTheta  = (deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM;

//        //this limit should match your loop time; 20°/cycle is safe for FTC.
        double maxTurnPerStep = Math.toRadians(10);
//
        if (isIMUUnhealthy(rawImuDeltaTheta, wheelDeltaTheta)) {
            if (shouldFallbackToWheelTheta) {
                //this is likely an IMU shock / 180° wrap → fall back to wheel
                imuDeltaTheta = wheelDeltaTheta;
                KLog.d("Odometry_IMU_Health", "Fallback Raw IMU " + rawImuDeltaTheta + " wheel " + wheelDeltaTheta);

            } else {
                KLog.d("Odometry_IMU_Health", "Fallback is off. NOT override Raw IMU " + rawImuDeltaTheta + " wheel " + wheelDeltaTheta);
            }
            KLog.d("Odometry_IMU_Health", "Raw IMU " + rawImuDeltaTheta + " wheel " + wheelDeltaTheta);
        } else {
            //IMU is usually better for heading, but we still keep 30% wheel
            // to improve continuity with the dead-wheel translation.
            imuDeltaTheta = rawImuDeltaTheta;
        }

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
        Position prevPosition = wheelPositionHistory.getCurrentPosition();
        Position globalPosition = calculateGlobal(wheelRelDelta, prevPosition);

        // Position jump filter - check for unrealistic movements
        double dx = globalPosition.getX() - prevPosition.getX();
        double dy = globalPosition.getY() - prevPosition.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        double maxSpeed = 1500; // mm/s (realistic max robot speed)
        double maxDistance = maxSpeed * timeElapsedSeconds;

        if (distance > maxDistance && timeElapsedSeconds > 0) {
            KLog.d("Odometry_Jump", "WHEEL position jump rejected: " + String.format("%.1f", distance) +
                   "mm in " + String.format("%.3f", timeElapsedSeconds) + "s (max: " +
                   String.format("%.1f", maxDistance) + "mm)");
            return; // Skip this update to prevent corruption
        }

        wheelPositionHistory.setCurrentPosition(globalPosition);
        wheelPositionHistory.setCurrentVelocity(wheelRelDelta, timeElapsedSeconds * 1000);
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL, wheelPositionHistory);
    }

    private void updateWheelIMUPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                   double timeElapsedSeconds) {

        Velocity wheelIMURelDelta = calculateRelativeDeltaWheelIMU(rightDistanceMM, leftDistanceMM, backDistanceMM,
                timeElapsedSeconds * 1000);
        wheelIMURelDelta = linearToArcDelta(wheelIMURelDelta);
        Position prevPosition = wheelIMUPositionHistory.getCurrentPosition();
        Position globalPosition = calculateGlobal(wheelIMURelDelta, prevPosition);

        // Position jump filter - check for unrealistic movements
        double dx = globalPosition.getX() - prevPosition.getX();
        double dy = globalPosition.getY() - prevPosition.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        double maxSpeed = 1500; // mm/s (realistic max robot speed)
        double maxDistance = maxSpeed * timeElapsedSeconds;

        if (distance > maxDistance && timeElapsedSeconds > 0) {
            KLog.d("Odometry_Jump", "WHEEL_IMU position jump rejected: " + String.format("%.1f", distance) +
                   "mm in " + String.format("%.3f", timeElapsedSeconds) + "s (max: " +
                   String.format("%.1f", maxDistance) + "mm)");
            return; // Skip this update to prevent corruption
        }

        wheelIMUPositionHistory.setCurrentPosition(globalPosition);
        wheelIMUPositionHistory.setCurrentVelocity(wheelIMURelDelta, timeElapsedSeconds * 1000); //mm/ms
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL_IMU, wheelIMUPositionHistory);
    }

    public HashMap<OdometrySensorCombinations, PositionHistory> updateAll() {
        if (!opModeUtilities.getOpMode().opModeIsActive()) {
            KLog.d("Odometry_debug_OpMode_Transfer", "OpMode Not Active. OpMode: " + opModeUtilities.getOpMode().getClass().getName());
            return odometryPositionHistoryHashMap;
        }

        // Clear bulk cache for MANUAL mode - ensures atomic encoder reads
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        // Read all encoders atomically - they now come from the same bulk read
        double rightDistanceMM = getRightEncoderMM();
        double leftDistanceMM = getLeftEncoderMM();
        double backDistanceMM = getBackEncoderMM();


        // Read IMU heading only once per cycle
        currentImuHeading = getIMUHeading();


        KLog.d("Odometry_IMU_Heading", "Heading (rad): " + currentImuHeading + " (deg): " + Math.toDegrees(currentImuHeading));
        KLog.d("Odometry_IMU_Prev_Heading", "PrevHeading (rad): " + prevImuHeading + " (deg): " + Math.toDegrees(prevImuHeading));


        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsedSeconds = (currentTime - prevTime) / 1000.0;

        updateWheelPos(rightDistanceMM,leftDistanceMM, backDistanceMM, timeElapsedSeconds);
        updateWheelIMUPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);

        prevTime = currentTime;

        prevRightDistanceMM = rightDistanceMM;
        prevLeftDistanceMM = leftDistanceMM;
        prevBackDistanceMM = backDistanceMM;
        Position wheelIMUPosition = odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU).getCurrentPosition();
        Position wheelPosition = odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL).getCurrentPosition();

        KLog.d("Odometry_IMU_Position", wheelIMUPosition.toString() + " UnhealthyCounter " + unhealthyCounter + " Unhealthyness " + isOdometryUnhealthy);
        KLog.d("Odometry_Wheel_Position", wheelPosition.toString() + " UnhealthyCounter " + unhealthyCounter + " Unhealthyness " + isOdometryUnhealthy);
        prevImuHeading = currentImuHeading;
        SharedData.setOdometryPosition(wheelIMUPosition);
        SharedData.setOdometryPositionMap(odometryPositionHistoryHashMap);
        SharedData.setIsOdometryUnhealthy(this.isOdometryUnhealthy);
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
        double imuHeading = -Math.toRadians(imuModule.getIMU().getRobotYawPitchRollAngles().getYaw());
        return imuHeading;
    }

    private boolean isBadReading(double reading) {
        if (Double.isNaN(reading) || Double.isInfinite(reading)) {
            return true;
        }
        return false;
    }

    public boolean isShouldFallbackToWheelTheta() {
        return shouldFallbackToWheelTheta;
    }

    public void setShouldFallbackToWheelTheta(boolean shouldFallbackToWheelTheta) {
        this.shouldFallbackToWheelTheta = shouldFallbackToWheelTheta;
    }


    private boolean isIMUUnhealthy(double imuThetaAngleRad, double wheelThetaAngleRad) {
        KLog.d("Odometry_Delta_Theta", "Checking IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
        if (unhealthyCounter > 2000 || isOutsideFieldBoundaries(getCurrentPositionHistory().getCurrentPosition())) {
            isOdometryUnhealthy = true;
        }
        boolean isSpike = (Math.abs(imuThetaAngleRad - wheelThetaAngleRad) > Math.toRadians(11)) && wheelThetaAngleRad != 0.0;
        if (isSpike) {
            unhealthyCounter++;
            KLog.d("Odometry_IMU_Health", "Spike IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
            return true;
        }
        boolean isFreezing = (Math.abs(imuThetaAngleRad) < 1e-10) && (Math.abs(wheelThetaAngleRad) > 1e-3);
        if (isFreezing) {
            unhealthyCounter++;
            KLog.d("Odometry_IMU_Health", "Freezing IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
            return true;
        }
        if (isBadReading(imuThetaAngleRad)) {
            unhealthyCounter++;
            KLog.d("Odometry_IMU_Health", "Bad Reading IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
            return true;
        }
        return false;
    }

    private boolean isOutsideFieldBoundaries(Position position) {
        if (Math.abs(position.getX()) > (3657.6 * 1.4) || Math.abs(position.getY()) > (3657.6 * 1.4)) {
            KLog.d("Odometry_IMU_Health", "Outside Field Boundaries" + position);
            return true;
        }
        return false;
    }

    public boolean isOdometryUnhealthy() {
        return isOdometryUnhealthy;
    }

    public void setOdometryUnhealthy(boolean odometryUnhealthy) {
        isOdometryUnhealthy = odometryUnhealthy;
    }

    @Override
    public String toString() {
        return "Odometry{" +
                "rightOffset=" + rightOffset +
                ", leftOffset=" + leftOffset +
                ", backOffset=" + backOffset +
                ", prevRightDistanceMM=" + prevRightDistanceMM +
                ", prevLeftDistanceMM=" + prevLeftDistanceMM +
                ", prevBackDistanceMM=" + prevBackDistanceMM +
                ", prevTime=" + prevTime +
                ", currentImuHeading=" + currentImuHeading +
                ", prevImuHeading=" + prevImuHeading +
                '}';
    }


}