package com.kalipsorobotics.localization;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.KLog;
import com.kalipsorobotics.utilities.OpModeUtilities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class OdometryLogger extends KFileWriter {

    private final List<String> bufferedLines = new ArrayList<>();
    private final Odometry odometry;
    private final boolean LOGGING_ENABLED = true;

    /**
     * Creates a new OdometryLogger <p>
     * Logs only WHEEL and WHEEL_IMU odometry calculations <p>
     * Data is buffered in memory and written to file when close() is called <p>
     * Use setLoggingEnabled(false) to disable logging for competition-ready code <p>
     */
    public OdometryLogger(String name, OpModeUtilities opModeUtilities, Odometry odometry) {
        super(name, opModeUtilities);
        this.odometry = odometry;
        writeHeader();
    }

    private void writeHeader() {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append("Time,");
        stringBuilder.append("Encoder_Back_Ticks,");
        stringBuilder.append("Encoder_Left_Ticks,");
        stringBuilder.append("Encoder_Right_Ticks,");
        stringBuilder.append("Encoder_Back_DistanceMM,");
        stringBuilder.append("Encoder_Left_DistanceMM,");
        stringBuilder.append("Encoder_Right_DistanceMM,");
        stringBuilder.append("RawIMUReadingRad,");
        stringBuilder.append("Wheel_X,");
        stringBuilder.append("Wheel_Y,");
        stringBuilder.append("Wheel_Theta,");
        stringBuilder.append("Wheel_DeltaTheta,");
        stringBuilder.append("Wheel_IMU_X,");
        stringBuilder.append("Wheel_IMU_Y,");
        stringBuilder.append("Wheel_IMU_Theta,");
        stringBuilder.append("Wheel_IMU_DeltaTheta,");
        stringBuilder.append("IMUUnhealthy");

        super.writeLine(stringBuilder.toString());
        try {
            super.flush();
        } catch (Exception e) {
            KLog.e("OdometryLogger", "Failed to flush header", e);
        }
    }

    /**
     * Logs the current odometry data to the buffer
     * @param positionHistoryHashMap HashMap containing WHEEL and WHEEL_IMU position histories
     */
    public void log(HashMap<OdometrySensorCombinations, PositionHistory> positionHistoryHashMap) {
        if (!LOGGING_ENABLED) {
            return;
        }

        PositionHistory wheelHistory = positionHistoryHashMap.get(OdometrySensorCombinations.WHEEL);
        PositionHistory wheelIMUHistory = positionHistoryHashMap.get(OdometrySensorCombinations.WHEEL_IMU);

        if (wheelHistory == null || wheelIMUHistory == null) {
            KLog.d("OdometryLogger", "Null position history detected, skipping log entry");
            return;
        }

        StringBuilder stringBuilder = new StringBuilder();

        // Time
        stringBuilder.append(System.currentTimeMillis());
        stringBuilder.append(",");

        // Encoder ticks (using wheel history as both should have same encoder values)
        stringBuilder.append(wheelHistory.getBackTicks());
        stringBuilder.append(",");
        stringBuilder.append(wheelHistory.getLeftTicks());
        stringBuilder.append(",");
        stringBuilder.append(wheelHistory.getRightTicks());
        stringBuilder.append(",");

        // Encoder distances in MM
        stringBuilder.append(wheelHistory.getBackDistanceMM());
        stringBuilder.append(",");
        stringBuilder.append(wheelHistory.getLeftDistanceMM());
        stringBuilder.append(",");
        stringBuilder.append(wheelHistory.getRightDistanceMM());
        stringBuilder.append(",");

        // Raw IMU reading in radians
        stringBuilder.append(wheelIMUHistory.getRawIMU());
        stringBuilder.append(",");

        // Wheel position data
        stringBuilder.append(wheelHistory.getCurrentPosition().getX());
        stringBuilder.append(",");
        stringBuilder.append(wheelHistory.getCurrentPosition().getY());
        stringBuilder.append(",");
        stringBuilder.append(wheelHistory.getCurrentPosition().getTheta());
        stringBuilder.append(",");
        stringBuilder.append(wheelHistory.getRelativeDelta().getTheta());
        stringBuilder.append(",");

        // Wheel IMU position data
        stringBuilder.append(wheelIMUHistory.getCurrentPosition().getX());
        stringBuilder.append(",");
        stringBuilder.append(wheelIMUHistory.getCurrentPosition().getY());
        stringBuilder.append(",");
        stringBuilder.append(wheelIMUHistory.getCurrentPosition().getTheta());
        stringBuilder.append(",");
        stringBuilder.append(wheelIMUHistory.getRelativeDelta().getTheta());
        stringBuilder.append(",");

        // IMU health status
        stringBuilder.append(odometry.isOdometryUnhealthy());

        bufferedLines.add(stringBuilder.toString());
        KLog.d("OdometryLogger", "Buffered line " + bufferedLines.size());
    }

    /**
     * Gets whether logging is enabled
     * @return true if logging is enabled
     */
    public boolean isLOGGING_ENABLED() {
        return LOGGING_ENABLED;
    }

    /**
     * Writes all buffered lines to the file and closes the writer
     * Call this before odometry shutdown
     */
    @Override
    public void close() {
        KLog.d("OdometryLogger", "Writing " + bufferedLines.size() + " buffered lines to file");
        for (String line : bufferedLines) {
            super.writeLine(line);
        }
        bufferedLines.clear();
        super.close();
    }
}
