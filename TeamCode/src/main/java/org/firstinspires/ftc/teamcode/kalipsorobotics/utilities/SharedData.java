package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.VisionRecognition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.HiveTagLocalizer;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometrySensorCombinations;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.PoseHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.PositionHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakData;
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
        targetCheckCounter++;
        if (targetCheckCounter >= ShooterConfig.TARGET_CHECK_EVERY_N_LOOPS) {
            targetCheckCounter = 0;
            targetPoint = pickTargetPoint(position.getY(), targetPoint);
        }
    }

    // ---- Hive Cell target: the one place that picks which Cell the shooter and turret aim at ----

    private static volatile Point targetPoint = null;
    private static int targetCheckCounter = 0;

    /**
     * The Hive Cell on the robot's end of the Hive: ShooterConfig.TARGET_POINT_1 (Cell A) when robot y is above
     * ShooterConfig.TARGET_SWITCH_Y_MM, TARGET_POINT_2 (Cell B) below it. Re-checked every ShooterConfig.TARGET_CHECK_EVERY_N_LOOPS odometry updates. Same numbers for
     * RED and BLUE: each alliance has its own frame (FieldConfig), so there is no polarity flip.
     */
    public static Point getTargetPoint() {
        Point target = targetPoint;
        if (target == null) { // nothing picked yet this OpMode: pick now from the current pose
            target = pickTargetPoint(odometryWheelIMUPosition.getY(), null);
            targetPoint = target;
        }
        return target;
    }

    /** Keeps the current Cell inside the deadband around the switch line. */
    private static Point pickTargetPoint(double robotY, Point current) {
        double switchY = ShooterConfig.TARGET_SWITCH_Y_MM;
        double deadband = ShooterConfig.TARGET_SWITCH_DEADBAND_MM;
        if (robotY > switchY + deadband) return ShooterConfig.TARGET_POINT_1;
        if (robotY < switchY - deadband) return ShooterConfig.TARGET_POINT_2;
        if (current != null) return current;
        return robotY >= switchY ? ShooterConfig.TARGET_POINT_1 : ShooterConfig.TARGET_POINT_2;
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
        targetPoint = null; // re-picked from the new pose on the next getTargetPoint()
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


    // Latest HIVE-tag fix (AprilTagDetectionAction). Written by the vision thread, read by the main loop.
    private static final Position limelightGlobalPosition = new Position(0, 0, 0);
    private static volatile long limelightFixNanos = 0;
    public static synchronized Position getLimelightGlobalPosition() {
        return new Position(limelightGlobalPosition);
    }
    public static synchronized void setLimelightGlobalPosition(Position position) {
        limelightGlobalPosition.reset(position);
        limelightFixNanos = System.nanoTime();
    }

    // How far odometry was off when the last fix was published (fix minus odometry, x/y). A rezero adds this to the
    // pose NOW, so a fix published while driving does not snap the robot back to where it was.
    private static double limelightOffsetX = 0;
    private static double limelightOffsetY = 0;
    /** Publishes the fix and its offset from odometry together (same timestamp). */
    public static synchronized void setLimelightFix(Position fix, double offsetX, double offsetY) {
        setLimelightGlobalPosition(fix);
        limelightOffsetX = offsetX;
        limelightOffsetY = offsetY;
    }
    /** {offsetX, offsetY} of the last fix; check getLimelightFixAgeMs() first. */
    public static synchronized double[] getLimelightOffset() {
        return new double[]{limelightOffsetX, limelightOffsetY};
    }
    /** Forget the last fix (after odometry jumps its offset is wrong): the fix age goes back to +Infinity. */
    public static synchronized void clearLimelightFix() {
        limelightFixNanos = 0;
        limelightOffsetX = 0;
        limelightOffsetY = 0;
    }

    // Bumped by every odometry reset (Limelight or corner), so the vision window drops offsets measured before the jump.
    private static volatile int odometryJumpCount = 0;
    public static void noteOdometryJump() {
        odometryJumpCount++;
    }
    public static int getOdometryJumpCount() {
        return odometryJumpCount;
    }
    /** Age of the last HIVE-tag fix in ms, or infinity if there never was one. */
    public static double getLimelightFixAgeMs() {
        return limelightFixNanos == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - limelightFixNanos) / 1e6;
    }

    // Goal state: which of our CELLs was last seen up. The HIVE only changes on a TIP, so the last sighting stands.
    private static volatile HiveTagLocalizer.Cell hiveUpCell = null;
    private static volatile long hiveUpCellNanos = 0;
    public static HiveTagLocalizer.Cell getHiveUpCell() {
        return hiveUpCell;
    }
    public static void setHiveUpCell(HiveTagLocalizer.Cell cell) {
        hiveUpCell = cell;
        hiveUpCellNanos = System.nanoTime();
    }
    public static double getHiveUpCellAgeMs() {
        return hiveUpCellNanos == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - hiveUpCellNanos) / 1e6;
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

    private static long lastVoltageReadMs = 0;
    /**
     * Re-reads the battery voltage at most every 500 ms (an I2C/Lynx read is slow). Called from both
     * KOpMode.updateActions (TeleOp) and Shooter.goToRPS (every flywheel loop, so autos that never call
     * updateActions still get KMotor.voltageScale compensation).
     */
    public static void refreshVoltage(HardwareMap hardwareMap) {
        long now = System.currentTimeMillis();
        if (now - lastVoltageReadMs < 500) {
            return;
        }
        lastVoltageReadMs = now;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = sensor.getVoltage();
            return;
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

    /** True while the Hive is moving (tipping). Path planning blocks the space under the Hive while it is true. */
    private static volatile boolean hiveTipping = false;

    public static void setHiveTipping(boolean tipping) {
        SharedData.hiveTipping = tipping;
    }

    public static boolean isHiveTipping() {
        return hiveTipping;
    }

    /** What one Cell of OUR Hive is doing: standing up, down, or swinging between the two. */
    public enum CellState { UP, DOWN, TIPPING }

    /** State of Cell A (the +y Cell), as the Limelight would sense it. null = nothing senses it. Cell B is the opposite (TIPPING stays TIPPING). */
    private static volatile CellState cellAState = null;

    public static void setCellAState(CellState state) {
        SharedData.cellAState = state;
    }

    public static CellState getCellAState() {
        return cellAState;
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

    private static volatile BeamBreakData beamBreakData = BeamBreakData.UNKNOWN;

    /**
     * How many balls the robot is holding, per the beam breaks. Published by BeamBreakArray every
     * time it is updated, which BeamBreakCountAction does once per loop.
     *
     * The snapshot is immutable and swapped in with one volatile write, so a subscriber on any
     * thread always reads a count and an occupancy pattern that came from the same sensor pass.
     * Never BeamBreakData.UNKNOWN once something has polled; check hasData() or isFresh() before
     * acting on it, since statics outlive an OpMode and the first reader of a new match could
     * otherwise see the last match's count.
     */
    public static BeamBreakData getBeamBreakData() {
        return beamBreakData;
    }

    public static void setBeamBreakData(BeamBreakData data) {
        beamBreakData = data == null ? BeamBreakData.UNKNOWN : data;
    }

    /** Shortcut for the common case. Returns 0 when nothing has been published yet. */
    public static int getBallCount() {
        return beamBreakData.getCount();
    }

    public static void resetBeamBreakData() {
        beamBreakData = BeamBreakData.UNKNOWN;
    }

    private static Point targetClusterXY = null;

    public static Point getTargetClusterXY() {
        return targetClusterXY;
    }

    public static void setTargetClusterXY(Point xy) {
        targetClusterXY = xy;
    }
}
