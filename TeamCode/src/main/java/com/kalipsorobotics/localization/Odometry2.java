//package com.kalipsorobotics.localization;
//
//import android.os.SystemClock;
//
//import com.kalipsorobotics.math.MathFunctions;
//import com.kalipsorobotics.modules.DriveTrain;
//import com.kalipsorobotics.utilities.KLog;
//
//import com.kalipsorobotics.math.PositionHistory;
//import com.kalipsorobotics.modules.GoBildaOdoModule;
//import com.kalipsorobotics.modules.IMUModule;
//import com.kalipsorobotics.utilities.OpModeUtilities;
//import com.kalipsorobotics.utilities.SharedData;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import com.kalipsorobotics.math.Position;
//import com.kalipsorobotics.math.Velocity;
//
//import java.util.HashMap;
//
//public class Odometry2 {
//
//    public enum OdometrySensorCombinations {
//        WHEEL,
//        WHEEL_IMU
//    }
//
//    private static Odometry single_instance = null;
//
//    // robot geometry
//    private static final double TRACK_WIDTH_MM = 333.0; // example value, your original value was here
//    private static final double BACK_DISTANCE_TO_MID_ROBOT_MM = 140.0; // example value, your original value was here
//
//    private final PositionHistory wheelPositionHistory;
//    private final PositionHistory wheelIMUPositionHistory;
//
//    private final HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionHistoryHashMap = new HashMap<>();
//
//    private final GoBildaOdoModule goBildaOdoModule;
//    private final IMUModule imuModule;
//
//    private double prevRightDistanceMM = 0;
//    private double prevLeftDistanceMM = 0;
//    private double prevBackDistanceMM = 0;
//
//    private double prevImuHeading = 0;
//    private double currentImuHeading = 0;
//
//    private long prevTime;
//
//    // region constructors / singleton
//
//    private Odometry2(OpModeUtilities opModeUtilities,
//                      DriveTrain driveTrain,
//                      IMUModule imuModule,
//                      Position startPosMMRAD) {
//
//        this.goBildaOdoModule = opModeUtilities.getGoBildaOdoModule();
//        this.imuModule = imuModule;
//
//        this.wheelPositionHistory = new PositionHistory(startPosMMRAD);
//        this.wheelIMUPositionHistory = new PositionHistory(startPosMMRAD);
//
//        this.prevTime = SystemClock.elapsedRealtime();
//
//        // initialize previous encoder distances
//        this.prevRightDistanceMM = getRightEncoderMM();
//        this.prevLeftDistanceMM = getLeftEncoderMM();
//        this.prevBackDistanceMM = getBackEncoderMM();
//
//        this.prevImuHeading = getIMUHeading();
//
//        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL, wheelPositionHistory);
//        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL_IMU, wheelIMUPositionHistory);
//    }
//
//    public static synchronized Odometry getInstance(OpModeUtilities opModeUtilities,
//                                                    DriveTrain driveTrain,
//                                                    IMUModule imuModule,
//                                                    Position startPosMMRAD) {
//        if (single_instance == null) {
//            single_instance = new Odometry(opModeUtilities, driveTrain, imuModule, startPosMMRAD);
//        } else {
//            resetHardware(driveTrain, imuModule, single_instance);
//        }
//        return single_instance;
//    }
//
//    private static void resetHardware(DriveTrain driveTrain, IMUModule imuModule, Odometry odometry) {
//        // reset logic if needed
//    }
//
//    // endregion
//
//    // region hardware reads
//
//    private double getRightEncoderMM() {
//        return goBildaOdoModule.getRightOdometryMM();
//    }
//
//    private double getLeftEncoderMM() {
//        return goBildaOdoModule.getLeftOdometryMM();
//    }
//
//    private double getBackEncoderMM() {
//        return goBildaOdoModule.getBackOdometryMM();
//    }
//
//    public double getIMUHeading() {
//        // original code
//        return -Math.toRadians(imuModule.getIMU().getRobotYawPitchRollAngles().getYaw());
//    }
//
//    // endregion
//
//    // region relative delta computations
//
//    // ----------------- WHEEL ONLY -----------------
//    private Velocity calculateRelativeDeltaWheel(double rightDistanceMM,
//                                                 double leftDistanceMM,
//                                                 double backDistanceMM,
//                                                 double deltaTimeMS) {
//        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
//        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
//        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;
//
//        // OLD CODE:
//        // double deltaTheta = MathFunctions.angleWrapDeg((deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM);
//        // ChatGPT recommendation: (ΔL - ΔR) / TRACK_WIDTH_MM already produces an angle in RADIANS
//        // because it is (arc length / radius). Wrapping in degrees here mixes units and can
//        // create small heading inconsistencies compared to the IMU branch.
//        double deltaTheta = MathFunctions.angleWrapRad((deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM);
//
//        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
//        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * deltaTheta);
//
//        Velocity velocity = new Velocity(deltaX, deltaY, deltaTheta);
//
//        return velocity;
//    }
//
//    // ----------------- WHEEL + IMU -----------------
//    private Velocity calculateRelativeDeltaWheelIMU(double rightDistanceMM,
//                                                    double leftDistanceMM,
//                                                    double backDistanceMM,
//                                                    double deltaTimeMS) {
//        double deltaRightDistance = rightDistanceMM - prevRightDistanceMM;
//        double deltaLeftDistance = leftDistanceMM - prevLeftDistanceMM;
//        double deltaMecanumDistance = backDistanceMM - prevBackDistanceMM;
//
//        // OLD CODE:
//        // double imuDeltaTheta = MathFunctions.angleWrapRad(currentImuHeading - prevImuHeading);
//
//        // ChatGPT recommendation: IMU can report large spikes (we saw ~±π in your CSV) when the
//        // robot slams the wall. Those are not real motions because you’re on dead-wheel odometry.
//        // So: compute the raw IMU delta, but clamp it to a physically realistic turn, and
//        // blend it with the wheel-based delta so heading stays stable.
//        double rawImuDeltaTheta = MathFunctions.angleWrapRad(currentImuHeading - prevImuHeading);
//        double wheelDeltaTheta  = (deltaLeftDistance - deltaRightDistance) / TRACK_WIDTH_MM;
//
//        // ChatGPT recommendation: this limit should match your loop time; 20°/cycle is safe for FTC.
//        double maxTurnPerStep = Math.toRadians(5);
//
//        double imuDeltaTheta;
//        if (Math.abs(rawImuDeltaTheta) > maxTurnPerStep) {
//            // ChatGPT recommendation: this is likely an IMU shock / 180° wrap → fall back to wheel
//            imuDeltaTheta = wheelDeltaTheta;
//        } else {
//            // ChatGPT recommendation: IMU is usually better for heading, but we still keep 30% wheel
//            // to improve continuity with the dead-wheel translation.
//            imuDeltaTheta = 0.7 * rawImuDeltaTheta + 0.3 * wheelDeltaTheta;
//        }
//
//        double deltaX = (deltaLeftDistance + deltaRightDistance) / 2;
//        double deltaY = (deltaMecanumDistance - BACK_DISTANCE_TO_MID_ROBOT_MM * imuDeltaTheta);
//
//        Velocity velocity = new Velocity(deltaX, deltaY, imuDeltaTheta);
//
//        return velocity;
//    }
//
//    // endregion
//
//    // region linear → arc, rotate, global
//
//    private Velocity linearToArcDelta(Velocity relativeDelta) {
//        if (Math.abs(relativeDelta.getTheta()) < 1e-4) {
//            return relativeDelta;
//        }
//
//        double forwardRadius = relativeDelta.getX() / relativeDelta.getTheta();
//        double strafeRadius = relativeDelta.getY() / relativeDelta.getTheta();
//
//        double relDeltaX =
//                forwardRadius * Math.sin(relativeDelta.getTheta()) +
//                        -strafeRadius * (1 - Math.cos(relativeDelta.getTheta()));
//
//        double relDeltaY =
//                strafeRadius * Math.sin(relativeDelta.getTheta()) +
//                        forwardRadius * (1 - Math.cos(relativeDelta.getTheta()));
//
//        double relDeltaTheta = relativeDelta.getTheta();
//
//        Velocity arcDelta = new Velocity(relDeltaX, relDeltaY, relDeltaTheta);
//        return arcDelta;
//    }
//
//    private Velocity rotate(Velocity relativeDelta, Position previousGlobalPosition) {
//        double sinTheta = Math.sin(previousGlobalPosition.getTheta());
//        double cosTheta = Math.cos(previousGlobalPosition.getTheta());
//
//        double globalX = relativeDelta.getX() * cosTheta - relativeDelta.getY() * sinTheta;
//        double globalY = relativeDelta.getX() * sinTheta + relativeDelta.getY() * cosTheta;
//
//        return new Velocity(globalX, globalY, relativeDelta.getTheta());
//    }
//
//    private Position calculateGlobal(Velocity relativeDelta, Position previousGlobalPosition) {
//        Velocity globalDelta = rotate(relativeDelta, previousGlobalPosition);
//        Position position = previousGlobalPosition.add(globalDelta);
//        return position;
//    }
//
//    // endregion
//
//    // region updates (called every loop)
//
//    private void updateWheelPos(double rightDistanceMM,
//                                double leftDistanceMM,
//                                double backDistanceMM,
//                                double timeElapsedSeconds) {
//
//        Velocity wheelRelDelta = calculateRelativeDeltaWheel(rightDistanceMM, leftDistanceMM,
//                backDistanceMM, timeElapsedSeconds * 1000);
//        wheelRelDelta = linearToArcDelta(wheelRelDelta);
//        Position globalPosition = calculateGlobal(wheelRelDelta, wheelPositionHistory.getCurrentPosition());
//        wheelPositionHistory.setCurrentPosition(globalPosition);
//
//        // this was already in seconds → keep it
//        wheelPositionHistory.setCurrentVelocity(wheelRelDelta, timeElapsedSeconds);
//
//        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL, wheelPositionHistory);
//    }
//
//    private void updateWheelIMUPos(double rightDistanceMM,
//                                   double leftDistanceMM,
//                                   double backDistanceMM,
//                                   double timeElapsedSeconds) {
//
//        Velocity wheelIMURelDelta = calculateRelativeDeltaWheelIMU(rightDistanceMM, leftDistanceMM, backDistanceMM,
//                timeElapsedSeconds * 1000);
//        wheelIMURelDelta = linearToArcDelta(wheelIMURelDelta);
//        Position globalPosition = calculateGlobal(wheelIMURelDelta, wheelIMUPositionHistory.getCurrentPosition());
//        wheelIMUPositionHistory.setCurrentPosition(globalPosition);
//
//        // ORIGINAL CODE:
//        // wheelIMUPositionHistory.setCurrentVelocity(wheelIMURelDelta, timeElapsedSeconds * 1000); //mm/ms
//        // ChatGPT recommendation: the wheel-only branch uses SECONDS for velocity; to keep
//        // downstream consumers consistent, use seconds here too.
//        wheelIMUPositionHistory.setCurrentVelocity(wheelIMURelDelta, timeElapsedSeconds);
//
//        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL_IMU, wheelIMUPositionHistory);
//    }
//
//    public HashMap<OdometrySensorCombinations, PositionHistory> updateAll() {
//        KLog.d("updatepos", "updatepos");
//
//        double rightDistanceMM = getRightEncoderMM();
//        double leftDistanceMM = getLeftEncoderMM();
//        double backDistanceMM = getBackEncoderMM();
//        currentImuHeading = getIMUHeading();
//
//        long currentTime = SystemClock.elapsedRealtime();
//        double timeElapsedSeconds = (currentTime - prevTime) / 1000.0;
//
//        updateWheelPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
//        updateWheelIMUPos(rightDistanceMM, leftDistanceMM, backDistanceMM, timeElapsedSeconds);
//
//        // update previous values for next call
//        prevRightDistanceMM = rightDistanceMM;
//        prevLeftDistanceMM = leftDistanceMM;
//        prevBackDistanceMM = backDistanceMM;
//        prevImuHeading = currentImuHeading;
//        prevTime = currentTime;
//
//        return odometryPositionHistoryHashMap;
//    }
//
//    // endregion
//
//    // region getters
//
//    public Position getWheelPosition() {
//        PositionHistory positionHistory = odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL);
//        if (positionHistory == null) {
//            throw new RuntimeException("WHEEL Position History Null");
//        }
//        return positionHistory.getCurrentPosition();
//    }
//
//    public Position getWheelIMUPosition() {
//        PositionHistory positionHistory = odometryPositionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU);
//        if (positionHistory == null) {
//            throw new RuntimeException("WHEEL_IMU Position History Null");
//        }
//        return positionHistory.getCurrentPosition();
//    }
//
//    private PositionHistory getCurrentPositionHistory() {
//        return wheelIMUPositionHistory;
//    }
//
//    // endregion
//}
