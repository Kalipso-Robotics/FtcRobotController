package com.kalipsorobotics.navigation;

import android.util.Log;

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
        StringBuilder stringBuilder = new StringBuilder();

        OdometrySensorCombinations key = OdometrySensorCombinations.WHEEL_IMU;

        stringBuilder.append("Time");
        stringBuilder.append(",");
        stringBuilder.append(key);
        stringBuilder.append(",");
        stringBuilder.append(key + "_X,");
        stringBuilder.append(key + "_Y,");
        stringBuilder.append(key + "_Theta,");
        stringBuilder.append(key + "_DeltaTheta,");

        stringBuilder.append("Motor_fLeft,");
        stringBuilder.append("Motor_fRight,");
        stringBuilder.append("Motor_bLeft,");
        stringBuilder.append("Motor_bRight");

        super.writeLine(stringBuilder.toString());
    }

    public void writeOdometryPositionHistory(PositionHistory positionHistory, DriveTrain driveTrain) {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(System.currentTimeMillis());
        stringBuilder.append(",");

        stringBuilder.append(OdometrySensorCombinations.WHEEL_IMU);
        stringBuilder.append(",");
        stringBuilder.append(positionHistory.toStringCSV());
        stringBuilder.append(",");

        stringBuilder.append(driveTrain.getfLeft().getPower());
        stringBuilder.append(",");
        stringBuilder.append(driveTrain.getfRight().getPower());
        stringBuilder.append(",");
        stringBuilder.append(driveTrain.getbLeft().getPower());
        stringBuilder.append(",");
        stringBuilder.append(driveTrain.getbRight().getPower());

        super.writeLine(stringBuilder.toString());
    }
    public void close() {
        super.close();
    }
}
