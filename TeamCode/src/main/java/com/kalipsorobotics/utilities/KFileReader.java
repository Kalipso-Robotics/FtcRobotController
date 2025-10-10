package com.kalipsorobotics.utilities;

import com.kalipsorobotics.utilities.KLog;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class KFileReader {
    /**
     * Reading Logs
     * <p>
     * alias pullLog='adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/OdometryLog ~/Downloads'
     *
     */

    OpModeUtilities opModeUtilities;
    String name;
    BufferedReader reader;


    public KFileReader(String fileName, OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.name = fileName;

        File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
        if(!path.exists()) {
            KLog.d("KFileReader", "Directory does not exist: " + path.getAbsolutePath());
            throw new RuntimeException("Directory does not exist: " + path.getAbsolutePath());
        }

        File file = new File(path, fileName);

        if (!file.exists()) {
            KLog.d("KFileReader", "File does not exist: " + file.getAbsolutePath());
            throw new RuntimeException("File does not exist: " + file.getAbsolutePath());
        }

        try {
            reader = new BufferedReader(new FileReader(file));
        } catch (IOException ioException) {
            KLog.e("IOException", "Failed Initializing File Reader For " + fileName,
                    ioException);
            throw new RuntimeException("Failed to initialize KFileReader for " + fileName, ioException);
        }
    }

    public String readLine() {
        try {
            return reader.readLine();
        } catch (IOException ioException) {
            KLog.d("IOException", "Caught IOException While Reading");
            return null;
        }
    }

    public void close() {
        try {
            reader.close();
        } catch (IOException e) {
            KLog.d("IOException", "Caught IOException While Closing");
        }
    }

    public BufferedReader getReader() {
        return reader;
    }
}
