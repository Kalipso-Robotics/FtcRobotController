package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.PositionHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class OdometryFileWriter extends KFileWriter {

    private final List<String> bufferedLines = new ArrayList<>();

    /**
     * Creates a new OdometryFileWriter <p>
     * name is at beginning of file <p>
     * .csv is always appended to name <p>
     * Write using writeOdometryPositionHistory() <p>
     * get OdometryPositionMap from SharedData <p>
     * Close using close() <p>
     * Data is buffered in memory and written to file when close() is called <p>
     */
    public OdometryFileWriter(String name, OpModeUtilities opModeUtilities) {
        super(name, opModeUtilities);
        writeOdometryHeader();
    }


    private void writeOdometryHeader() {
        StringBuilder stringBuilder = new StringBuilder();

        stringBuilder.append("Time");
        stringBuilder.append(",");
        for (OdometrySensorCombinations key : OdometrySensorCombinations.values()) {
            stringBuilder.append(key.toString());
            stringBuilder.append(",");
            stringBuilder.append(key + "_X,");
            stringBuilder.append(key + "_Y,");
            stringBuilder.append(key + "_Theta,");
            stringBuilder.append(key + "_DeltaTheta,");
            stringBuilder.append(key + "_LeftTicks,");
            stringBuilder.append(key + "_RightTicks,");
            stringBuilder.append(key + "_BackTicks,");
            stringBuilder.append(key + "_Action,");

        }
        super.writeLine(stringBuilder.toString());
        try {
            super.flush();
        } catch (Exception e) {
            KLog.e("OdometryFileWriter", "Failed to flush header", e);
        }
    }
    public void writeOdometryPositionHistory(HashMap<OdometrySensorCombinations, PositionHistory> positionHistoryHashMap, String action) {
        KLog.d("odometryData", "writeOdometryPositionHistory" + positionHistoryHashMap.toString());
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(System.currentTimeMillis());
        stringBuilder.append(",");

        for (OdometrySensorCombinations key : OdometrySensorCombinations.values()) {

            stringBuilder.append(key.toString());
            stringBuilder.append(",");

            if (positionHistoryHashMap.get(key) == null) {
                stringBuilder.append(PositionHistory.toNullString());
            } else {
                stringBuilder.append(positionHistoryHashMap.get(key).toStringCSV());
            }

            stringBuilder.append(",");

        }
        stringBuilder.append(action);
        KLog.d("odometry_file_writer_line", stringBuilder.toString());
        bufferedLines.add(stringBuilder.toString());
    }
    public void writeOdometryPositionHistory(HashMap<OdometrySensorCombinations, PositionHistory> positionHistoryHashMap) {
        writeOdometryPositionHistory(positionHistoryHashMap, "");
    }
    public void close() {
        KLog.d("OdometryFileWriter", "Writing " + bufferedLines.size() + " buffered lines to file");
        for (String line : bufferedLines) {
            super.writeLine(line);
        }
        bufferedLines.clear();
        super.close();
    }
}
