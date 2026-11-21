package org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.List;

/**
 * PLACEHOLDER. Commit 5b208324 uses HiveTagLocalizer, but the real file was never pushed, so master does not compile.
 * This stub has only the API the callers use. solve() always returns "no fix", so vision localization and
 * Hive up/down sensing are OFF until the real file replaces this one (delete this file when it arrives).
 */
public final class HiveTagLocalizer {

    /** True while this is the placeholder: TeleOp and LimeLightTest show a warning line. */
    public static final boolean IS_PLACEHOLDER = true;
    private static boolean warned = false;

    private HiveTagLocalizer() {
    }

    public enum Cell { A, B }

    public static final class Fix {
        public final Point robot;     // null = no robot position from the tags
        public final Cell upCell;     // null = Hive state not seen
        public final int tagsUsed;

        public Fix(Point robot, Cell upCell, int tagsUsed) {
            this.robot = robot;
            this.upCell = upCell;
            this.tagsUsed = tagsUsed;
        }
    }

    /** PLACEHOLDER: always "no fix". */
    public static Fix solve(List<TagObservation> observations, double headingRad, double turretAngleRad, AllianceColor allianceColor) {
        if (!warned) {
            warned = true;
            org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog.e("Vision_Localizer",
                    "event=placeholder HiveTagLocalizer is a stub: no Limelight fix, no Hive state, no auto pass");
        }
        return new Fix(null, null, 0);
    }
}
