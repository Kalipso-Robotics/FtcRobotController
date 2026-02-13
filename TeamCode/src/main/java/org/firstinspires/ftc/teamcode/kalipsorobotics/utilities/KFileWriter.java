package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class KFileWriter {
    /**
     * Deleting and Pulling Logs
     * <p>
     * alias deleteLog=adb shell "rm -r /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/OdometryLog/*"
     * <p>
     * alias pullLog='adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/OdometryLog ~/
     * '
     *
     */

    OpModeUtilities opModeUtilities;
    // Get the current date and time
    Date now;
    SimpleDateFormat formatter;
    String formattedDateTime;

    String name;
    BufferedWriter writer;


    public KFileWriter(String name, OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.name = name;
        now = new Date();
        formatter = new SimpleDateFormat("yyyy_MM_dd__HH_mm_ss_SSS", Locale.US);
        formattedDateTime = formatter.format(now);

        File path = new File(opModeUtilities.getHardwareMap().appContext.getExternalFilesDir(null), "OdometryLog");
        if(!path.exists()) {
            if (!path.mkdirs()) {
                KLog.d("KFileWriter", "Failed To Make Directory");
                throw new RuntimeException("Failed To Make Directory");
            }
        }

        File file = new File(path, name + "_" + formattedDateTime + ".csv");

        try {
            writer = new BufferedWriter(new FileWriter(file));
        } catch (IOException ioException) {
            KLog.e("KFileWriter", "Failed Initializing File Writer For " + name + "_" + formattedDateTime + ".csv",
                    ioException);
            throw new RuntimeException("Failed to initialize KFileWriter for " + name + "_" + formattedDateTime + ".csv", ioException);
        }


    }

    public void writeLine(String string) {
        try {
            writer.write(string);
            writer.newLine();
        } catch (IOException ioException) {
            KLog.e("KFileWriter", "Failed to write line to " + name, ioException);
        }
    }

    public void flush() throws IOException {
        writer.flush();
    }

    public void close() {
        try {
            writer.flush();
            writer.close();
        } catch (IOException e) {
            KLog.e("KFileWriter", "Failed to close file writer for " + name, e);
        }
    }

    public BufferedWriter getWriter() {
        return writer;
    }
}
