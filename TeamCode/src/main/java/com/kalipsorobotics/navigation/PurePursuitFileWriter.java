package com.kalipsorobotics.navigation;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.OpModeUtilities;

import java.util.HashMap;

public class PurePursuitFileWriter extends KFileWriter {

    /**
     * Creates a new OdometryFileWriter <p>
     * name is at beginning of file <p>
     * .csv is always appended to name <p>
     * Write using writeOdometryPositionHistory() <p>
     * get OdometryPositionMap from SharedData <p>
     * Close using close() <p>
     */
    public PurePursuitFileWriter(String name, OpModeUtilities opModeUtilities) {
        super(name, opModeUtilities);
        writeOdometryHeader();
    }


    private void writeOdometryHeader() {

        OdometrySensorCombinations key = OdometrySensorCombinations.WHEEL_IMU;

        String stringBuilder = "Time" +
                "," +
                key +
                "," +
                key + "_X," +
                key + "_Y," +
                key + "_Theta," +
                key + "_DeltaTheta," +
                "Motor_fLeft," +
                "Motor_fRight," +
                "Motor_bLeft," +
                "Motor_bRight";

        super.writeLine(stringBuilder);
    }

    public void writePurePursuitData(HashMap<OdometrySensorCombinations, PositionHistory> positionHistoryMap, DriveTrain driveTrain) {

        String stringBuilder = System.currentTimeMillis() +
                "," +
                OdometrySensorCombinations.WHEEL_IMU +
                "," +
                positionHistoryMap.get(OdometrySensorCombinations.WHEEL_IMU).toStringCSV() +
                "," +
                driveTrain.getfLeftPower() +
                "," +
                driveTrain.getfRightPower() +
                "," +
                driveTrain.getbLeftPower() +
                "," +
                driveTrain.getbRightPower();
        KLog.d("purepursaction_power", "power " + driveTrain.getfLeftPower() + " " + driveTrain.getfRightPower() + " " + driveTrain.getbLeftPower() + " " + driveTrain.getbRightPower());
        super.writeLine(stringBuilder);
    }

    public void close() {
        super.close();
    }

}
