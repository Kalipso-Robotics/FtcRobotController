package com.kalipsorobotics.localization;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.OpModeUtilities;

import java.util.HashMap;

public class OdometryFileWriter extends KFileWriter {

    /**
     * Creates a new OdometryFileWriter <p>
     * name is at beginning of file <p>
     * .csv is always appended to name <p>
     * Write using writeOdometryPositionHistory() <p>
     * get OdometryPositionMap from SharedData <p>
     * Close using close() <p>
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

        }
        super.writeLine(stringBuilder.toString());
    }

    public void writeOdometryPositionHistory(HashMap<OdometrySensorCombinations, PositionHistory> positionHistoryHashMap) {
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
        super.writeLine(stringBuilder.toString());
    }
    public void close() {
        super.close();
    }
}
