package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import android.util.Log;

import java.util.function.Supplier;

public class KLog {

    private final static boolean DEBUG = false;

    public static boolean isDebug() {
        return DEBUG;
    }

    /**
     * Preferred overload for any log message that involves string concatenation or method calls.
     * The lambda body is never evaluated when DEBUG = false, so zero string construction
     * overhead occurs in the main loop.
     *
     * Usage: KLog.d("tag", () -> "val: " + someGetter());
     */
    public static void d(String tag, Supplier<String> message) {
        if (DEBUG) {
            Log.d("KLog_" + tag, message.get());
        }
    }

    /**
     * Use only for plain string literals where no concatenation or method calls occur.
     * Plain literals are compile-time constants and are already zero-cost.
     */
    public static void d(String tag, String message) {
        if (DEBUG) {
            Log.d("KLog_" + tag, message);
        }
    }

    public static void e(String tag, String message, Exception e) {
        if (DEBUG) {
            Log.e("KLog_" + tag, message, e);
        }
    }

    public static void e(String tag, String message) {
        if (DEBUG) {
            Log.e("KLog_" + tag, message);
        }
    }


}
