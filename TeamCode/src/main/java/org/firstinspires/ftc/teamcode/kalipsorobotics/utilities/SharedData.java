package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometrySensorCombinations;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.PoseHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.LimelightPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.PositionHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SharedData {

    private static final Position odometryWheelIMUPosition = new Position(0, 0, 0);

    /** Returns a defensive copy. Safe for storing or mutating. */
    public static Position getOdometryWheelIMUPosition() {
        return new Position(odometryWheelIMUPosition);
    }

    /** Returns a direct reference - zero allocation. Use only for read-only access within a single loop iteration. */
    public static Position peekOdometryWheelIMUPosition() {
        return odometryWheelIMUPosition;
    }
    /** Records the pose as of now. Resets and externally-set poses enter the history as a jump. */
    public static void setOdometryWheelIMUPosition(Position position) {
        setOdometryWheelIMUPosition(position, System.nanoTime());
    }

    /** sampleNanos is when the sensors behind this pose were read (System.nanoTime()). */
    public static void setOdometryWheelIMUPosition(Position position, long sampleNanos) {
        odometryWheelIMUPosition.reset(position);
        odometryWheelIMUPoseHistory.record(sampleNanos, position);
    }

    private static final PoseHistory odometryWheelIMUPoseHistory = new PoseHistory(128);

    /**
     * Pose at a past time, e.g. a camera frame's captureTimeNanos. Null if that is older than the
     * history; callers should then fall back to the latest pose and log it.
     */
    public static Position getOdometryWheelIMUPositionAt(long nanos) {
        return odometryWheelIMUPoseHistory.at(nanos);
    }
    /** Fitted v/a over the last windowMs of odometry. Null if no samples yet. */
    public static PoseHistory.Motion getOdometryWheelIMUMotion(long windowMs) {
        return odometryWheelIMUPoseHistory.fit(windowMs * 1_000_000L);
    }

    /** Blocks until odometry records a sample newer than afterNanos, or timeoutMs passes. */
    public static boolean awaitOdometrySampleAfter(long afterNanos, long timeoutMs) throws InterruptedException {
        return odometryWheelIMUPoseHistory.awaitNewer(afterNanos, timeoutMs);
    }

    private static volatile SOTM.Solution sotmSolution = null;
    private static volatile int sotmActiveGoal = 0;

    public static SOTM.Solution getSOTMSolution() {
        return sotmSolution;
    }
    public static void setSOTMSolution(SOTM.Solution solution) {
        sotmSolution = solution;
    }
    public static int getSOTMActiveGoal() {
        return sotmActiveGoal;
    }
    public static void setSOTMActiveGoal(int goalIndex) {
        sotmActiveGoal = goalIndex;
    }

    public static void resetOdometryWheelIMUPosition() {
        odometryWheelIMUPosition.reset(new Position(0, 0, 0));
    }


    private static final Position odometryWheelPosition = new Position(0, 0, 0);

    public static Position getOdometryWheelPosition() {
        return new Position(odometryWheelPosition);
    }
    public static void setOdometryWheelPosition(Position position) {
        odometryWheelPosition.reset(position);
    }
    public static void resetOdometryWheelPosition() {
        odometryWheelPosition.reset(new Position(0, 0, 0));
    }



    private static final HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionMap = new HashMap<>();
    public static HashMap<OdometrySensorCombinations, PositionHistory> getOdometryPositionMap() {
        return new HashMap<>(odometryPositionMap);
    }
    public static void setOdometryPositionMap(HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionMap) {
        SharedData.odometryPositionMap.putAll(odometryPositionMap);
    }


    private static final LimelightPos limelightRawPosition = new LimelightPos(0,0,0,0,0);
    public static void setLimelightRawPosition(LimelightPos position) {
        limelightRawPosition.setPos(position);
    }
    public static LimelightPos getLimelightRawPosition() {
        return limelightRawPosition;
    }


    private static final Position limelightGlobalPosition = new Position(0, 0, 0);
    public static Position getLimelightGlobalPosition() {
        return new Position(limelightGlobalPosition);
    }
    public static void setLimelightGlobalPosition(Position position) {
        limelightGlobalPosition.reset(position);
    }


    private static long unhealthyCounter = 0;
    public static long getUnhealthyCounter() {
        return unhealthyCounter;
    }
    public static void setUnhealthyCounter(long unhealthyCounter) {
        SharedData.unhealthyCounter = unhealthyCounter;
    }


    //For transfer between Auto and TeleOp
    private static AllianceColor allianceColor = AllianceColor.RED;

    public static AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public static void setAllianceColor(AllianceColor allianceColor) {
        SharedData.allianceColor = allianceColor;
    }

    private static final Velocity odometryWheelVelocity = new Velocity(0, 0, 0);

    public static Velocity getOdometryWheelVelocity() {
        return new Velocity(odometryWheelVelocity);
    }
    public static void setOdometryWheelVelocity(Velocity velocity) {
        odometryWheelVelocity.reset(velocity);
    }
    public static void resetOdometryWheelVelocity() {
        odometryWheelVelocity.reset(new Velocity(0, 0, 0));
    }

    private static final Velocity odometryWheelIMUVelocity = new Velocity(0, 0, 0);

    /** Returns a defensive copy. Safe for storing or mutating. */
    public static Velocity getOdometryWheelIMUVelocity() {
        return new Velocity(odometryWheelIMUVelocity);
    }

    /** Returns a direct reference - zero allocation. Use only for read-only access within a single loop iteration. */
    public static Velocity peekOdometryWheelIMUVelocity() {
        return odometryWheelIMUVelocity;
    }
    public static void setOdometryWheelIMUVelocity(Velocity velocity) {
        odometryWheelIMUVelocity.reset(velocity);
    }
    public static void resetOdometryWheelIMUVelocity() {
        odometryWheelIMUVelocity.reset(new Velocity(0, 0, 0));
    }

    private static double voltage = ShooterInterpolationConfig.DEFAULT_VOLTAGE;
    public static double getVoltage() {
        return voltage;
    }
    public static void setVoltage(double newVoltage) {
        voltage = newVoltage;
    }

    private static final Position unfilteredLimelightGlobalPos = new Position(0,0,0);
    public static Position getUnfilteredLimelightGlobalPos() {
        return new Position(unfilteredLimelightGlobalPos);
    }

    public static void setUnfilteredLimelightGlobalPos(Position position) {
        if (position != null) {
            unfilteredLimelightGlobalPos.reset(position);
        } else {
            unfilteredLimelightGlobalPos.reset(new Position(0,0,0));
        }
    }

    private static volatile List<VisionRecognition> pollenNectarDetections = new ArrayList<>();
    private static volatile long pollenNectarDetectionsTimeMs = 0;


    public static void setPollenNectarDetections(List<VisionRecognition> pollenNectarDetections) {
        SharedData.pollenNectarDetections = new ArrayList<>(pollenNectarDetections);
        pollenNectarDetectionsTimeMs = System.currentTimeMillis();
    }


    public static List<VisionRecognition> getPollenNectarDetections() {
        return new ArrayList<>(pollenNectarDetections);
    }


    public static long getPollenNectarDetectionsTimeMs() {
        return pollenNectarDetectionsTimeMs;
    }


    public static void resetPollenNectarDetections() {
        pollenNectarDetections = new ArrayList<>();
        pollenNectarDetectionsTimeMs = 0;
    }

    private static volatile List<BallInformation> ballInformation = new ArrayList<>();
    private static volatile long ballInformationTimeMs = 0;

    /** Balls from the latest raytraced frame, field frame in mm. Published every frame, empty included. */
    public static void setBallInformation(List<BallInformation> balls) {
        SharedData.ballInformation = new ArrayList<>(balls);
        ballInformationTimeMs = System.currentTimeMillis();
    }

    public static List<BallInformation> getBallInformation() {
        return new ArrayList<>(ballInformation);
    }

    public static long getBallInformationTimeMs() {
        return ballInformationTimeMs;
    }

    public static void resetBallInformation() {
        ballInformation = new ArrayList<>();
        ballInformationTimeMs = 0;
    }
}
