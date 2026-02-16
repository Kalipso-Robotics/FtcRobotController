package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import android.util.Log;

public class KLog {

    private final static boolean DEBUG = false;

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
