package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import android.os.SystemClock;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.ExponentialVelocityFiltering;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.PositionHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometryConfig.alphaLinear;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometryConfig.alphaTheta;

/**
 * Drop-in for {@link Odometry} that takes its motion from the OctoQuad's on-board localizer.
 * Same shape: singleton, {@code updateAll()}/{@code update()}, publishes to the WHEEL_IMU
 * SharedData slot, so every consumer (DriveAction, PurePursuit, turret, resets) is unchanged.
 * Switching an OpMode is {@code Odometry -> OctoQuadOdo}; {@code OpModeUtilities
 * .runOdometryExecutorService} has an overload for it.
 *
 * Like Odometry, each tick starts from the previous pose in SharedData and adds a delta, so
 * ResetOdometryToPos / ResetOdometryToLimelight and the auto start pose work without ever
 * writing to the board. The delta is the change in the board's own pose, rotated by the constant
 * offset between the board's heading and ours.
 *
 * The board is configured and its IMU calibrated only on a FRESH instance (robot must be still).
 * A reused instance (Auto -> TeleOp) leaves the board alone so its heading carries over.
 * A missing board or failed calibration throws from getInstance, failing the OpMode at init.
 */
public class OctoQuadOdo {

    private static OctoQuadOdo single_instance = null;

    private OpModeUtilities opModeUtilities;
    private OctoQuad board;

    private final OctoQuad.LocalizerDataBlock localizerData = new OctoQuad.LocalizerDataBlock();
    private final OctoConfig.Pose boardPose = new OctoConfig.Pose();

    private final PositionHistory wheelIMUPositionHistory = new PositionHistory();
    private final HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionHistoryHashMap =
            new HashMap<>();
    private final ExponentialVelocityFiltering emaWheelIMU =
            new ExponentialVelocityFiltering(alphaLinear, alphaTheta);

    // Board pose (robot convention) at the last valid read.
    private double prevBoardX, prevBoardY, prevBoardThetaRad;
    private long prevTime;
    private long unhealthyCounter = 0;

    /** One executor tick, for diagnostics. Immutable, so a reader on another thread never sees a
     *  board pose from one tick next to our pose from another. Board pose is robot frame, rad. */
    public static final class Tick {
        public final long count;
        public final double boardX, boardY, boardThetaRad;
        public final Position pose;
        /** Time since the previous valid tick started, and time spent in the I2C read. */
        public final double intervalMs, readMs;

        Tick(long count, double boardX, double boardY, double boardThetaRad, Position pose,
             double intervalMs, double readMs) {
            this.count = count;
            this.boardX = boardX;
            this.boardY = boardY;
            this.boardThetaRad = boardThetaRad;
            this.pose = pose;
            this.intervalMs = intervalMs;
            this.readMs = readMs;
        }
    }

    private volatile Tick lastTick;
    private long tickCount;
    private long lastTickNanos;

    private OctoQuadOdo(OpModeUtilities opModeUtilities, Position startPosMMRad) {
        KLog.d("Octoquad_debug_OpMode_Transfer", "New Instance");
        this.opModeUtilities = opModeUtilities;
        initBoard();

        wheelIMUPositionHistory.setCurrentPosition(startPosMMRad);
        wheelIMUPositionHistory.setCurrentVelocity(new Velocity(0, 0, 0), 0);
        odometryPositionHistoryHashMap.put(OdometrySensorCombinations.WHEEL_IMU, wheelIMUPositionHistory);

        SharedData.setOdometryPositionMap(odometryPositionHistoryHashMap);
        SharedData.setOdometryWheelIMUPosition(wheelIMUPositionHistory.getCurrentPosition());

        if (!readBoard()) {
            throw new IllegalStateException("OctoQuad returned invalid localizer data right after calibration");
        }
        prevTime = SystemClock.elapsedRealtime();
    }

    public static synchronized OctoQuadOdo getInstance(OpModeUtilities opModeUtilities) {
        return getInstance(opModeUtilities, new Position(0, 0, 0));
    }

    public static synchronized OctoQuadOdo getInstance(OpModeUtilities opModeUtilities, Position startPosMMRad) {
        if (single_instance == null) {
            single_instance = new OctoQuadOdo(opModeUtilities, startPosMMRad);
        } else {
            KLog.d("Octoquad_debug_OpMode_Transfer", () -> "Reuse Instance" + single_instance);
            single_instance.opModeUtilities = opModeUtilities;
            single_instance.board = lookUpBoard(opModeUtilities);
        }
        return single_instance;
    }

    public static synchronized OctoQuadOdo getInstance(OpModeUtilities opModeUtilities,
                                                       double startXMM, double startYMM, double startThetaRad) {
        return getInstance(opModeUtilities, new Position(startXMM, startYMM, startThetaRad));
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static OctoQuad lookUpBoard(OpModeUtilities opModeUtilities) {
        try {
            return opModeUtilities.getHardwareMap().get(OctoQuad.class, OctoConfig.HARDWARE_NAME);
        } catch (RuntimeException e) {
            throw new IllegalStateException("OctoQuad '" + OctoConfig.HARDWARE_NAME
                    + "' not found in the RC config", e);
        }
    }

    /** Push constants, then zero the localizer and calibrate the IMU. Robot must be still. */
    private void initBoard() {
        board = lookUpBoard(opModeUtilities);
        OctoConfig.apply(board);
        String error = OctoConfig.calibrateImu(board,
                () -> opModeUtilities.getOpMode().isStopRequested(), null);
        if (error != null) {
            throw new IllegalStateException("OctoQuad: " + error);
        }
    }

    /** Reads the board into prevBoard*; false (and a counted failure) if the block is invalid. */
    private boolean readBoard() {
        board.readLocalizerData(localizerData);
        if (!localizerData.isDataValid()) {
            unhealthyCounter++;
            SharedData.setUnhealthyCounter(unhealthyCounter);
            KLog.d("Octoquad_Update", () -> "invalid localizer data, crcOk=" + localizerData.crcOk
                    + " status=" + localizerData.localizerStatus);
            return false;
        }
        OctoConfig.toRobotFrame(localizerData, boardPose);
        prevBoardX = boardPose.x;
        prevBoardY = boardPose.y;
        prevBoardThetaRad = Math.toRadians(boardPose.headingDeg);
        return true;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    /** The raw SDK board, for calibration and raw reads. */
    public OctoQuad getBoard() {
        return board;
    }

    public HashMap<OdometrySensorCombinations, PositionHistory> updateAll() {
        if (!opModeUtilities.getOpMode().opModeIsActive()) {
            return odometryPositionHistoryHashMap;
        }

        long sampleNanos = System.nanoTime();
        long currentTime = SystemClock.elapsedRealtime();
        double timeElapsedMS = currentTime - prevTime;
        double lastBoardX = prevBoardX, lastBoardY = prevBoardY, lastBoardTheta = prevBoardThetaRad;
        if (!readBoard()) {
            return odometryPositionHistoryHashMap; // hold last pose, failure already counted
        }
        double readMs = (System.nanoTime() - sampleNanos) / 1e6;
        prevTime = currentTime;

        // Re-read each tick so resets written to SharedData (ResetOdometryToPos etc.) take effect.
        Position prev = SharedData.getOdometryWheelIMUPosition();

        // Board field delta -> our field frame: constant rotation by (our heading - board heading).
        // Same rotation matrix as Odometry.rotate().
        double offset = prev.getTheta() - lastBoardTheta;
        double dx = prevBoardX - lastBoardX;
        double dy = prevBoardY - lastBoardY;
        double sin = Math.sin(offset), cos = Math.cos(offset);
        double fieldDx = dx * cos - dy * sin;
        double fieldDy = dy * cos + dx * sin;
        double dTheta = MathFunctions.angleWrapRad(prevBoardThetaRad - lastBoardTheta);

        Position pos = prev.add(new Velocity(fieldDx, fieldDy, dTheta));

        if (timeElapsedMS > 0) {
            Velocity filtered = emaWheelIMU.calculateFilteredVelocity(
                    new Velocity(fieldDx / timeElapsedMS, fieldDy / timeElapsedMS, dTheta / timeElapsedMS));
            SharedData.setOdometryWheelIMUVelocity(filtered); // mm/ms, field frame
        }

        // Robot-relative delta, for PositionHistory's relative velocity (same as Odometry).
        double sinP = Math.sin(prev.getTheta()), cosP = Math.cos(prev.getTheta());
        Velocity relDelta = new Velocity(fieldDx * cosP + fieldDy * sinP,
                fieldDy * cosP - fieldDx * sinP, dTheta);

        wheelIMUPositionHistory.setRawIMU(prevBoardThetaRad);
        wheelIMUPositionHistory.setCurrentPosition(pos);
        wheelIMUPositionHistory.setCurrentVelocity(relDelta, timeElapsedMS);

        SharedData.setOdometryWheelIMUPosition(pos, sampleNanos);
        SharedData.setOdometryPositionMap(odometryPositionHistoryHashMap);
        SharedData.setUnhealthyCounter(unhealthyCounter);

        double intervalMs = lastTickNanos == 0 ? 0 : (sampleNanos - lastTickNanos) / 1e6;
        lastTickNanos = sampleNanos;
        lastTick = new Tick(++tickCount, prevBoardX, prevBoardY, prevBoardThetaRad, pos, intervalMs, readMs);
        return odometryPositionHistoryHashMap;
    }

    /** Latest executor tick, or null before the first one. */
    public Tick getLastTick() {
        return lastTick;
    }

    public Position update() {
        return updateAll().get(OdometrySensorCombinations.WHEEL_IMU).getCurrentPosition();
    }
}
