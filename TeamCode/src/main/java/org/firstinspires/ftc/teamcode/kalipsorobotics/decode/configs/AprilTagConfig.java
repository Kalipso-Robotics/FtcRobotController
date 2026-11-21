package org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

@Config
public class AprilTagConfig {


    // DECODE goal point. Only Shooter.TARGET_POINT (used by the old decode/auto OpModes) still reads these two.
    public static double APRILTAG_X_REL_FIELD_MM = 3076.6; // 3115 FROM TUNE | 3,076.6 FROM MEASURE FIELD WITH TAPE MEASURE
    public static double APRILTAG_Y_REL_FIELD_MM = 1013.5; //951.43 Tuned until zero | 1,013.5 mm IS FROM MEASURED FIELD WITH TAPE MEASURE

    // ---- Limelight mount (measured, not Dashboard-tuned) ----

    /** Camera on the turret, turret frame (x forward, y right, theta = camera yaw vs the turret). TODO-measure: DECODE value. */
    public static final Position TURRET_REL_CAM_POS = new Position(-147.565, 0.977, 0);
    /** Lens height above the tiles. TODO-measure */
    public static final double CAM_HEIGHT_MM = 350;
    /** Camera tilt above horizontal (the tags are overhead). TODO-measure */
    public static final double CAM_PITCH_DEG = 25;

    // ---- BIOBUZZ tags (manual 9.6, 9.9, Figures 9-9, 9-10, 9-15, 9-17) ----
    // 36h11, 3.25 in, four per CELL on the CELL's underside. Set the Limelight pipeline to 36h11, size 82.55 mm.
    // A HIVE tips +-30 deg about its pivot, so each tag has one 3D spot while its CELL is up and another while it is
    // down. The measured tag height tells the two apart (they are ~360 mm apart), which gives both the robot position
    // and the goal state (which CELL can be scored in).
    // Cell A is the high-y CELL of an alliance's own frame, B the low-y one (see FieldConfig).
    // Red: audience side 34-37 (A), far side 30-33 (B). Blue: far side 42-45 (A), audience side 38-41 (B).

    public static int[] cellATagIds(AllianceColor allianceColor) {
        return allianceColor == AllianceColor.RED ? new int[]{34, 35, 36, 37} : new int[]{42, 43, 44, 45};
    }

    public static int[] cellBTagIds(AllianceColor allianceColor) {
        return allianceColor == AllianceColor.RED ? new int[]{30, 31, 32, 33} : new int[]{38, 39, 40, 41};
    }

    private static final double IN = 25.4;
    private static final double FIELD_MM = 144 * IN;
    public static final double HIVE_PIVOT_Z_MM = 43.95 * IN;
    public static final double HIVE_TILT_RAD = Math.toRadians(30);
    /** Each HIVE's pivot is this far from the field center along x (25.5 in center to center). */
    public static final double HIVE_X_FROM_CENTER_MM = 25.5 / 2 * IN;
    /** Tag centerline from the pivot along the level arm: half the 18.84 in gap + 12.04 in CELL - 9.938 in (front to holes) + 2.75 in (holes to tags). */
    public static final double TAG_ALONG_ARM_MM = (18.84 / 2 + 12.04 - 9.938 + 2.75) * IN;
    /** CELL underside below the pivot axis, level. VERIFY: 3.0 in from the 30.6 in "bottom of HIVE", 1.3 in from the 53.5 in "bottom of opening". */
    public static final double TAG_BELOW_PIVOT_MM = 2.2 * IN;
    /** Tag centers across the cluster, in printed order (lowest ID first), cluster center = CELL centerline. */
    private static final double[] TAG_ACROSS_MM = {-6.5 * IN, -2.75 * IN, 2.75 * IN, 6.5 * IN};

    // ---- Gates ----

    /** A tag whose measured height is further than this from both its up and down heights is ignored (HIVE tipping, bad solve). */
    public static double TAG_HEIGHT_TOLERANCE_MM = 120;
    /** All tags in one frame must put the robot within this of their average, or the frame gives no position. */
    public static double MAX_TAG_SPREAD_MM = 150;
    /** A Limelight fix older than this is not used to re-zero odometry. */
    public static double MAX_FIX_AGE_MS = 150;

    // ---- Sliding window over camera frames (AprilTagDetectionAction) ----

    /** How many of the latest camera frames (with our tags in them) are remembered. */
    public static final int HIVE_WINDOW_FRAMES = 5;
    /** Frames older than this no longer count (the robot looked away, or the camera was off). */
    public static double HIVE_WINDOW_MAX_AGE_MS = 300;
    /** A Limelight result older than this when we read it is thrown away (stale cached frame). */
    public static double MAX_LIMELIGHT_STALENESS_MS = 100;
    /** A CELL is published as up once this many frames in the window say so and none say the other CELL. */
    public static int HIVE_WINDOW_MIN_AGREE = 3;
    /** Tipping is published once this many frames in the window say tipping. */
    public static int HIVE_WINDOW_MIN_TIPPING = 2;
    /** A robot position is published once this many frames in the window have one (their average is published). */
    public static int HIVE_WINDOW_MIN_FIXES = 2;

    /**
     * Tag center in an alliance's own frame, {x, y, z} mm, or null if the ID is not a HIVE tag.
     * Built in the RED frame (manual top view, origin top-left, +x right, +y down = Figure 9-17 as printed);
     * BLUE is the same field rotated 180 deg. VERIFY against the real field.
     */
    public static double[] tagFieldXYZ(int id, boolean cellUp, AllianceColor allianceColor) {
        if (id < 30 || id > 45) return null;
        int cluster = (id - 30) / 4; // 0 red far, 1 red audience, 2 blue audience, 3 blue far
        int index = (id - 30) % 4;
        boolean red = cluster < 2;
        boolean far = cluster == 0 || cluster == 3;

        // Figure 9-17 (top view): far clusters read high ID -> low ID left to right, audience clusters low -> high.
        double across = far ? -TAG_ACROSS_MM[index] : TAG_ACROSS_MM[index];
        double x = FIELD_MM / 2 + (red ? -HIVE_X_FROM_CENTER_MM : HIVE_X_FROM_CENTER_MM) + across;

        double c = Math.cos(HIVE_TILT_RAD), s = Math.sin(HIVE_TILT_RAD);
        double out = cellUp ? TAG_ALONG_ARM_MM * c + TAG_BELOW_PIVOT_MM * s : TAG_ALONG_ARM_MM * c - TAG_BELOW_PIVOT_MM * s;
        double z = HIVE_PIVOT_Z_MM + (cellUp ? TAG_ALONG_ARM_MM * s : -TAG_ALONG_ARM_MM * s) - TAG_BELOW_PIVOT_MM * c;
        double y = FIELD_MM / 2 + (far ? -out : out);

        // RED keeps the numbers (polarity +1); BLUE flips them around the field center (polarity -1 = 180 deg turn).
        int polarity = allianceColor.getPolarity();
        x = FIELD_MM / 2 + polarity * (x - FIELD_MM / 2);
        y = FIELD_MM / 2 + polarity * (y - FIELD_MM / 2);
        return new double[]{x, y, z};
    }

    /** 'A' or 'B' if the tag is on one of this alliance's own CELLs, else 0. */
    public static char ownCellOf(int id, AllianceColor allianceColor) {
        for (int a : cellATagIds(allianceColor)) if (a == id) return 'A';
        for (int b : cellBTagIds(allianceColor)) if (b == id) return 'B';
        return 0;
    }
}
