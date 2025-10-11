package com.kalipsorobotics.utilities;

import android.util.Log;

public class KLog {

    private final static boolean DEBUG = true;

    public static void d(String tag, String message) {
        if (DEBUG) {
            Log.d(tag, message);
        }
    }

    public static void e(String tag, String message, Exception e) {
        if (DEBUG) {
            Log.e(tag, message, e);
        }
    }

    public static void e(String tag, String message) {
        if (DEBUG) {
            Log.e(tag, message);
        }
    }


}
