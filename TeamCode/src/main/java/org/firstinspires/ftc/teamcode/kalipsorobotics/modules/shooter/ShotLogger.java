package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import java.util.ArrayList;
import java.util.List;

/**
 * Logs shooter performance data during teleop matches.
 * Accumulates data in memory and writes to file at end of match to minimize overhead.
 */
public class ShotLogger {

    public enum ShotQuality {
        GOOD("Good"),
        BAD_UNDERSHOT("Bad - Undershot"),
        BAD_OVERSHOT("Bad - Overshot");

        private final String description;

        ShotQuality(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    public static class ShotData {
        public final double targetRPS;
        public final double actualRPS;
        public final double hoodPosition;
        public final double distanceMM;
        public ShotQuality quality;
        public final long timestamp;

        public ShotData(double targetRPS, double actualRPS, double hoodPosition, double distanceMM) {
            this.targetRPS = targetRPS;
            this.actualRPS = actualRPS;
            this.hoodPosition = hoodPosition;
            this.distanceMM = distanceMM;
            this.quality = ShotQuality.GOOD;
            this.timestamp = System.currentTimeMillis();
        }

        public void markAsUndershot() {
            this.quality = ShotQuality.BAD_UNDERSHOT;
        }

        public void markAsOvershot() {
            this.quality = ShotQuality.BAD_OVERSHOT;
        }

        @Override
        public String toString() {
            return String.format("Target RPS: %.2f, Actual RPS: %.2f, Hood: %.3f, Distance: %.1fmm, Quality: %s",
                targetRPS, actualRPS, hoodPosition, distanceMM, quality.getDescription());
        }

        public String toCsvLine() {
            return String.format("%.2f,%.2f,%.3f,%.1f,%s,%d",
                targetRPS, actualRPS, hoodPosition, distanceMM, quality.getDescription(), timestamp);
        }
    }

    private final List<ShotData> shots;
    private ShotData lastShot;
    private KFileWriter fileWriter;
    private final OpModeUtilities opModeUtilities;

    public ShotLogger(OpModeUtilities opModeUtilities) {
        this.shots = new ArrayList<>();
        this.lastShot = null;
        this.opModeUtilities = opModeUtilities;
        this.fileWriter = null;
    }

    /**
     * Log a new shot. Defaults to "good" quality.
     * Logs to KLog immediately for live monitoring.
     */
    public void logShot(double targetRPS, double actualRPS, double hoodPosition, double distanceMM) {
        ShotData shot = new ShotData(targetRPS, actualRPS, hoodPosition, distanceMM);
        shots.add(shot);
        lastShot = shot;

        // Log to KLog immediately
        KLog.d("ShotLogger", String.format("Shot #%d: %s", shots.size(), shot));
    }

    /**
     * Mark the last shot as undershot.
     */
    public void markLastShotAsUndershot() {
        if (lastShot != null) {
            lastShot.markAsUndershot();
            KLog.d("ShotLogger", String.format("Shot #%d marked as UNDERSHOT", shots.size()));
        }
    }

    /**
     * Mark the last shot as overshot.
     */
    public void markLastShotAsOvershot() {
        if (lastShot != null) {
            lastShot.markAsOvershot();
            KLog.d("ShotLogger", String.format("Shot #%d marked as OVERSHOT", shots.size()));
        }
    }

    /**
     * Write all accumulated shot data to file at end of match.
     * Only called once to minimize overhead during the match.
     */
    public void writeToFile() {
        if (shots.isEmpty()) {
            KLog.d("ShotLogger", "No shots to write to file");
            return;
        }

        try {
            fileWriter = new KFileWriter("ShotLog", opModeUtilities);

            // Write CSV header
            fileWriter.writeLine("TargetRPS,ActualRPS,HoodPosition,DistanceMM,Quality,Timestamp");

            // Write all shot data
            for (ShotData shot : shots) {
                fileWriter.writeLine(shot.toCsvLine());
            }

            // Flush to ensure all data is written to disk before closing
            try {
                fileWriter.flush();
            } catch (Exception flushException) {
                KLog.e("ShotLogger", "Failed to flush data to file", flushException);
            }

            fileWriter.close();

            KLog.d("ShotLogger", String.format("Successfully wrote %d shots to file", shots.size()));
        } catch (Exception e) {
            KLog.e("ShotLogger", "Failed to write shots to file", e);
            if (fileWriter != null) {
                fileWriter.close();
            }
        }
    }

    /**
     * Get the number of shots logged.
     */
    public int getShotCount() {
        return shots.size();
    }
}