package com.kalipsorobotics.localization;

import static com.kalipsorobotics.decode.configs.DrivetrainConfig.*;
import static com.kalipsorobotics.localization.OdometryConfig.alpha;

import android.os.SystemClock;

import com.kalipsorobotics.math.ExponentialVelocityFiltering;
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

import java.util.HashMap;



public class Odometry {

    private long unhealthyCounter = 0;
    private boolean shouldFallbackToWheelTheta = true;

    private static Odometry single_instance = null;
    final private PositionHistory wheelPositionHistory = new PositionHistory();
    final private PositionHistory wheelIMUPositionHistory = new PositionHistory();

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    private OpModeUtilities opModeUtilities;
    HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionHistoryHashMap = new HashMap<>();
    IMUModule imuModule;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;

    private int currentLeftTicks = 0;

    private int currentRightTicks = 0;
    private int currentBackTicks = 0;
    private final double rightOffset;
    private final double leftOffset;
    private final double backOffset;
    private double prevRightDistanceMM;
    private double prevLeftDistanceMM;
    volatile private double prevBackDistanceMM;
    private long prevTime;
    private double currentImuHeading;
    private double prevImuHeading;
    private Position prevPositionWheel;
    private Position prevPositionWheelIMU;
    private Velocity robotWheelIMUVelocity;
    private Velocity robotWheelVelocity;
    private final ExponentialVelocityFiltering ema;
    double rightDistanceMM;
    double leftDistanceMM;
    double backDistanceMM;

    long currentTime = SystemClock.elapsedRealtime();
    double timeElapsedMS = (currentTime - prevTime);


    private Odometry(OpModeUtilities opModeUtilities, DriveTrain driveTrain, IMUModule imuModule,
                     Position startPosMMRad) {
        KLog.d("Odometry_debug_OpMode_Transfer", "New Instance");
        this.opModeUtilities = opModeUtilities;
        resetHardware(opModeUtilities, driveTrain, imuModule, this);
        this.rightOffset = this.getRightEncoderMM();
        this.leftOffset = this.getLeftEncoderMM();
        this.backOffset = this.getBackEncoderMM();
        this.ema = new ExponentialVelocityFiltering(alpha);

        this.wheelPositionHistory.setCurrentPosition(startPosMMRad);
        this.wheelIMUPositionHistory.setCurrentPosition(startPosMMRad);

        this.wheelPositionHistory.setCurrentVelocity(new Velocity(0, 0, 0), 0);
        this.wheelIMUPositionHistory.setCurrentVelocity(new Velocity(0, 0, 0), 0);

        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL, wheelPositionHistory);
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL_IMU, wheelIMUPositionHistory);

        SharedData.setOdometryPositionMap(odometryPositionHistoryHashMap);
        SharedData.setOdometryWheelPosition(wheelPositionHistory.getCurrentPosition());
        SharedData.setOdometryWheelIMUPosition(wheelIMUPositionHistory.getCurrentPosition());

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
            KLog.d("Auto→TeleOp", "Creating NEW odometry instance at (0,0,0)");
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        } else {
            KLog.d("Auto→TeleOp", "REUSING existing odometry instance");
            KLog.d("Odometry_debug_OpMode_Transfer", "Reuse Instance" + single_instance.toString());
            KLog.d("Auto→TeleOp", "Position before resetHardware: " + single_instance.wheelIMUPositionHistory.getCurrentPosition());
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
            KLog.d("Auto→TeleOp", "Position after resetHardware: " + single_instance.wheelIMUPositionHistory.getCurrentPosition());
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, Position startPosMMRad) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, startPosMMRad);
        } else {
            KLog.d("Odometry_debug_OpMode_Transfer", "Reuse Instance" + single_instance);
            resetHardware(opModeUtilities, driveTrain, imuModule, single_instance);
        }
        return single_instance;
    }

    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities, DriveTrain driveTrain,
                                                    IMUModule imuModule, double startXMM, double startYMM, double startThetaRad) {
        if (single_instance == null) {
            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, startXMM, startYMM, startThetaRad);
        } else {
            KLog.d("Odometry_debug_OpMode_Transfer", "Reuse Instance" + single_instance);
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
    }


    public static void setInstanceNull() {
        single_instance = null;
    }

    private static double ticksToMM(double ticks) {
        return ticks * MM_PER_TICK;
    }
    private static int mmToTicks(double mm) {
        return (int) (mm / MM_PER_TICK);
    }

    public static double getTrackWidthMm() {
        return TRACK_WIDTH_MM;
    }

    public double getRightEncoderMM() {
        //corresponds to fRight
        //direction FORWARD
        //negative because encoder directions
        currentRightTicks = -rightEncoder.getCurrentPosition();
        KLog.d("Odometry_Encoder", "right encoder " + currentRightTicks);
        return ticksToMM(currentRightTicks) - rightOffset;
        //return ticksToMM(rightEncoder.getCurrentPosition());
    }
    public double getLeftEncoderMM() {
        //corresponds to fLeft
        //direction FORWARD
        //positive because encoder directions
        currentLeftTicks = leftEncoder.getCurrentPosition();
        KLog.d("Odometry_Encoder", "left encoder " + currentLeftTicks);
        return ticksToMM(currentLeftTicks) - leftOffset;
    }
    public double getBackEncoderMM() {
        //corresponds to bRight
        //direction REVERSE
        //positive because encoder directions
        currentBackTicks = backEncoder.getCurrentPosition();
        KLog.d("Odometry_Encoder", "back encoder " + currentBackTicks);
        return ticksToMM(currentBackTicks) - backOffset;
        //return ticksToMM(backEncoder.getCurrentPosition());
    }

    public boolean allEncoderZero() {
        return (currentRightTicks == 0) && (currentLeftTicks == 0) && (currentBackTicks == 0);
    }

    public boolean anyEncoderZero() {
        return (currentRightTicks == 0) || (currentLeftTicks == 0) || (currentBackTicks == 0);
    }

    private Velocity calculateRelativeDeltaWheel(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        double deltaTheta = MathFunctions.angleWrapRad((deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM);

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * deltaTheta);
        //BackDistanceToMid = (backDistance - deltaY)/deltaTheta

        Velocity deltaPos = new Velocity(deltaX, deltaY, deltaTheta);

        Velocity rawVelocity = new Velocity(deltaX / deltaTimeMS, deltaY / deltaTimeMS, deltaTheta / deltaTimeMS);

        robotWheelVelocity = ema.calculateFilteredVelocity(rawVelocity);

        KLog.d("Odometry_WheelVelocity", "Raw Velocity: " + rawVelocity +
                " Filtered Velocity: " + robotWheelVelocity);


        return deltaPos;
    }

    private Velocity calculateRelativeDeltaWheelIMU(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double deltaTimeMS) {
        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;

        KLog.d("Odometry_Delta_Theta", "Raw IMU Current Heading" + currentImuHeading + " Prev IMU Heading " + prevImuHeading);

        // Ensure both headings are properly wrapped before computing delta
        double wrappedCurrentHeading = MathFunctions.angleWrapRad(currentImuHeading);
        double wrappedPrevHeading = MathFunctions.angleWrapRad(prevImuHeading);
        double rawImuDeltaTheta = MathFunctions.angleWrapRad(wrappedCurrentHeading - wrappedPrevHeading);

        double imuDeltaTheta = rawImuDeltaTheta;
        double wheelDeltaTheta  = MathFunctions.angleWrapRad((deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM);
        KLog.d("Odometry_Delta_Theta", "Imu Delta Theta: " + rawImuDeltaTheta + " Wheel Delta Theta: " + wheelDeltaTheta);

        if (isIMUUnhealthy(rawImuDeltaTheta, wheelDeltaTheta)) {
            if (shouldFallbackToWheelTheta) {
                //this is likely an IMU shock / 180° wrap → fall back to wheel
                imuDeltaTheta = wheelDeltaTheta;
                KLog.d("Odometry_IMU_Health", "Fallback Raw IMU " + rawImuDeltaTheta + " wheel " + wheelDeltaTheta);

            } else {
                imuDeltaTheta = rawImuDeltaTheta;
                KLog.d("Odometry_IMU_Health", "Fallback is off. NOT override Raw IMU " + rawImuDeltaTheta + " wheel " + wheelDeltaTheta);
            }
        } else {
            //IMU is usually better for heading, but we still keep 30% wheel
            // to improve continuity with the dead-wheel translation.
            imuDeltaTheta = rawImuDeltaTheta;
            KLog.d("Odometry_IMU_Health", "Healthy Raw IMU " + rawImuDeltaTheta + " wheel " + wheelDeltaTheta);
        }

        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * imuDeltaTheta);

        Velocity deltaPos = new Velocity(deltaX, deltaY, imuDeltaTheta);

        Velocity rawVelocity = new Velocity(deltaX / deltaTimeMS, deltaY / deltaTimeMS, imuDeltaTheta / deltaTimeMS);


        KLog.d("Odometry_WheelIMUVelocity", "Raw Velocity: " + rawVelocity +
                " Filtered Velocity: " + robotWheelVelocity);

        return deltaPos;
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

    //TODO Ask claude to verify and change global calculation later
    private Position calculateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);
        //KLog.d("global delta", globalDelta.toString());
        Position position = previousGlobalPosition.add(globalDelta);
        return position;
    }

    private void updateWheelPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM, double timeElapsedMS) {
        Velocity wheelRelDelta = calculateRelativeDeltaWheel(rightDistanceMM, leftDistanceMM,
                backDistanceMM, timeElapsedMS);
        wheelRelDelta = linearToArcDelta(wheelRelDelta);
        Position globalPosition = calculateGlobal(wheelRelDelta, prevPositionWheel);

        Position deltaPos = globalPosition.calculateDelta(prevPositionWheel);
        Velocity rawVelocity = new Velocity(deltaPos.getX() / timeElapsedMS, deltaPos.getY() / timeElapsedMS, deltaPos.getTheta() / timeElapsedMS);
        robotWheelVelocity = ema.calculateFilteredVelocity(rawVelocity);

        wheelPositionHistory.setRawIMU(currentImuHeading);
        wheelPositionHistory.setDistanceMM(leftDistanceMM, rightDistanceMM, backDistanceMM);
        wheelPositionHistory.setCurrentPosition(globalPosition);
        wheelPositionHistory.setCurrentVelocity(wheelRelDelta, timeElapsedMS);
        wheelPositionHistory.setEncoderTicks(currentLeftTicks,
                currentRightTicks, currentBackTicks);
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL, wheelPositionHistory);
    }

    private void updateWheelIMUPos(double rightDistanceMM, double leftDistanceMM, double backDistanceMM,
                                   double timeElapsedMS) {
        Velocity wheelIMURelDelta = calculateRelativeDeltaWheelIMU(rightDistanceMM, leftDistanceMM, backDistanceMM,
                timeElapsedMS);
        wheelIMURelDelta = linearToArcDelta(wheelIMURelDelta);
        Position globalPosition = calculateGlobal(wheelIMURelDelta, prevPositionWheelIMU);

        Position deltaPos = globalPosition.calculateDelta(prevPositionWheelIMU);
        Velocity rawVelocity = new Velocity(deltaPos.getX() / timeElapsedMS, deltaPos.getY() / timeElapsedMS, deltaPos.getTheta() / timeElapsedMS);
        robotWheelIMUVelocity = ema.calculateFilteredVelocity(rawVelocity);

        wheelIMUPositionHistory.setRawIMU(currentImuHeading);
        wheelIMUPositionHistory.setDistanceMM(leftDistanceMM, rightDistanceMM, backDistanceMM);
        wheelIMUPositionHistory.setCurrentPosition(globalPosition);
        wheelIMUPositionHistory.setCurrentVelocity(wheelIMURelDelta, timeElapsedMS); //mm/ms
        wheelIMUPositionHistory.setEncoderTicks(currentLeftTicks,
                currentRightTicks, currentBackTicks);
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL_IMU, wheelIMUPositionHistory);
    }

    private void readData() {

        // Read all encoders atomically - they now come from the same bulk read
        rightDistanceMM = getRightEncoderMM();
        leftDistanceMM = getLeftEncoderMM();
        backDistanceMM = getBackEncoderMM();

        // Read IMU heading only once per cycle
        currentImuHeading = getIMUHeading();
        KLog.d("Odometry_IMU_Heading", "Heading (rad): " + currentImuHeading + " (deg): " + Math.toDegrees(currentImuHeading));
        KLog.d("Odometry_IMU_Prev_Heading", "PrevHeading (rad): " + prevImuHeading + " (deg): " + Math.toDegrees(prevImuHeading));
        currentTime = SystemClock.elapsedRealtime();
        timeElapsedMS = (currentTime - prevTime);
        prevPositionWheel = SharedData.getOdometryWheelPosition();
        prevPositionWheelIMU = SharedData.getOdometryWheelIMUPosition();
    }

    private HashMap<OdometrySensorCombinations, PositionHistory> updateResult() {

        prevTime = currentTime;

        prevRightDistanceMM = rightDistanceMM;
        prevLeftDistanceMM = leftDistanceMM;
        prevBackDistanceMM = backDistanceMM;
        Position wheelIMUPosition = odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU).getCurrentPosition();
        Position wheelPosition = odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL).getCurrentPosition();

        KLog.d("Odometry_IMU_Position", wheelIMUPosition.toString() + " UnhealthyCounter " + unhealthyCounter);
        KLog.d("Odometry_Wheel_Position", wheelPosition.toString() + " UnhealthyCounter " + unhealthyCounter);
        prevImuHeading = currentImuHeading;
        SharedData.setOdometryWheelIMUPosition(wheelIMUPosition);
        SharedData.setOdometryWheelPosition(wheelPosition);

        SharedData.setOdometryPositionMap(odometryPositionHistoryHashMap);
        SharedData.setUnhealthyCounter(unhealthyCounter);

        SharedData.setOdometryWheelIMUVelocity(robotWheelIMUVelocity);
        SharedData.setOdometryWheelVelocity(robotWheelVelocity);

        return odometryPositionHistoryHashMap;
    }

    public HashMap<OdometrySensorCombinations, PositionHistory> updateAll() {
        if (!opModeUtilities.getOpMode().opModeIsActive()) {
            KLog.d("Odometry_debug_OpMode_Transfer", "OpMode Not Active. OpMode: " + opModeUtilities.getOpMode().getClass().getName());
            return odometryPositionHistoryHashMap;
        }

        readData();

        if (allEncoderZero()) {
            KLog.d("Odometry_debug_OpMode_Transfer", "All encoders zero. Returning..");
            return odometryPositionHistoryHashMap;
        }


        updateWheelPos(rightDistanceMM,leftDistanceMM, backDistanceMM, timeElapsedMS);
        updateWheelIMUPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedMS);


        HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionHistoryMap = updateResult();
        return odometryPositionHistoryMap;
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
        return Double.isNaN(reading) || Double.isInfinite(reading);
    }

    public boolean isShouldFallbackToWheelTheta() {
        return shouldFallbackToWheelTheta;
    }

    public void setShouldFallbackToWheelTheta(boolean shouldFallbackToWheelTheta) {
        this.shouldFallbackToWheelTheta = shouldFallbackToWheelTheta;
    }


    private boolean isIMUUnhealthy(double imuThetaAngleRad, double wheelThetaAngleRad) {
        KLog.d("Odometry_Delta_Theta", "Checking IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
        if (unhealthyCounter > 20000  || isOutsideFieldBoundaries(getCurrentPositionHistory().getCurrentPosition())) {
        }
        if (isBadReading(imuThetaAngleRad)) {
            unhealthyCounter++;
            KLog.d("Odometry_IMU_Unhealthy", "Bad Reading IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
            return true;
        }
        double angleDiff = MathFunctions.angleWrapRad(imuThetaAngleRad - wheelThetaAngleRad);
        boolean isSpike = (Math.abs(angleDiff) > Math.toRadians(11));
        if (isSpike) {
            unhealthyCounter++;
            KLog.d("Odometry_IMU_Unhealthy", "Spike IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
            return true;
        }
        boolean isFreezing = (Math.abs(imuThetaAngleRad) < 1e-10) && (Math.abs(wheelThetaAngleRad) > 1e-3);
        if (isFreezing) {
            unhealthyCounter++;
            KLog.d("Odometry_IMU_Unhealthy", "Freezing IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
            return true;
        }
        boolean  isIMUPhantom = (Math.abs(wheelThetaAngleRad) == 0) && (Math.abs(imuThetaAngleRad) != 0);
        if (isIMUPhantom) {
            unhealthyCounter++;
            KLog.d("Odometry_IMU_Unhealthy", "IMUPhantom IMU delta theta " + imuThetaAngleRad + " wheel delta theta " + wheelThetaAngleRad);
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


    public int getCurrentRightTicks() {
        return currentRightTicks;
    }

    public int getCurrentBackTicks() {
        return currentBackTicks;
    }

    public int getCurrentLeftTicks() {
        return currentLeftTicks;
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