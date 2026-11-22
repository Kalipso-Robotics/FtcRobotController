package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Locale;

/**
 * Live (real-time) driving. SimRunner calls pace() before every step, so virtual time never runs ahead of the wall clock.
 *
 * stdin, one command per line:
 *   gp <1|2> lx ly rx ry lt rt buttons   sticks clamped to [-1,1], triggers to [0,1]; buttons = bitmask:
 *                                        0 a, 1 b, 2 x, 3 y, 4 lb, 5 rb, 6 up, 7 down, 8 left, 9 right, 10 back, 11 start, 12 guide, 13 ls, 14 rs
 *   set <ConfigClass>.<FIELD> <value>    a public static number/boolean in decode.configs (what the Dashboard would change)
 *   stop                                 ends the session (so does end of input)
 * stdout: "#frame {json}" every 20 ms, "#event seconds|text", "#telemetry {json}", "#set ok|error ...".
 */
public final class SimLive {

    static final String CONFIG_PACKAGE = "org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.";

    private volatile boolean stopRequested;
    private volatile boolean pacing;
    private long simStartNanos;
    private long wallStartNanos;

    boolean isStopRequested() {
        return stopRequested;
    }

    boolean isPacing() {
        return pacing;
    }

    /** From now on virtual time simNanos is "now" on the wall clock. */
    void startPacing(long simNanos) {
        simStartNanos = simNanos;
        wallStartNanos = System.nanoTime();
        pacing = true;
    }

    /** Sleeps until the wall clock reaches virtual time toNanos. Before START (init) the sim runs as fast as it can. */
    void pace(long toNanos) {
        if (!pacing) {
            return;
        }
        long ahead = (toNanos - simStartNanos) - (System.nanoTime() - wallStartNanos);
        if (ahead > 1_000_000L) {
            try {
                Thread.sleep(ahead / 1_000_000L, (int) (ahead % 1_000_000L));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    void startReader(InputStream in, Gamepad gamepad1, Gamepad gamepad2) {
        Thread reader = new Thread(() -> {
            try (BufferedReader lines = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {
                for (String line = lines.readLine(); line != null && !stopRequested; line = lines.readLine()) {
                    handle(line.trim(), gamepad1, gamepad2);
                }
            } catch (Exception ignored) {
                // a closed pipe ends the session like "stop"
            }
            stopRequested = true;
        }, "sim-live-stdin");
        reader.setDaemon(true);
        reader.start();
    }

    void handle(String line, Gamepad gamepad1, Gamepad gamepad2) {
        if (line.equals("stop")) {
            stopRequested = true;
        } else if (line.startsWith("set ")) {
            String[] v = line.split(" ");
            String reply;
            try {
                reply = "#set ok " + setConfig(v[1], v[2]);
            } catch (Exception e) {
                reply = "#set error " + (v.length > 1 ? v[1] : "") + " " + e.getMessage();
            }
            System.out.println(reply);
            System.out.flush();
        } else {
            applyLine(line, gamepad1, gamepad2);
        }
    }

    /** Applies one "gp" line. Returns false (and changes nothing) for anything malformed. */
    static boolean applyLine(String line, Gamepad gamepad1, Gamepad gamepad2) {
        String[] v = line.split(" ");
        if (v.length != 9 || !v[0].equals("gp") || !(v[1].equals("1") || v[1].equals("2"))) {
            return false;
        }
        float[] f = new float[6];
        int b;
        try {
            for (int i = 0; i < 6; i++) {
                f[i] = Float.parseFloat(v[i + 2]);
                if (!Float.isFinite(f[i])) {
                    return false;
                }
            }
            b = Integer.parseInt(v[8]);
        } catch (NumberFormatException e) {
            return false;
        }
        Gamepad g = v[1].equals("2") ? gamepad2 : gamepad1;
        g.left_stick_x = clamp(f[0], -1);
        g.left_stick_y = clamp(f[1], -1);
        g.right_stick_x = clamp(f[2], -1);
        g.right_stick_y = clamp(f[3], -1);
        g.left_trigger = clamp(f[4], 0);
        g.right_trigger = clamp(f[5], 0);
        g.a = (b & 1) != 0;
        g.b = (b & 1 << 1) != 0;
        g.x = (b & 1 << 2) != 0;
        g.y = (b & 1 << 3) != 0;
        g.left_bumper = (b & 1 << 4) != 0;
        g.right_bumper = (b & 1 << 5) != 0;
        g.dpad_up = (b & 1 << 6) != 0;
        g.dpad_down = (b & 1 << 7) != 0;
        g.dpad_left = (b & 1 << 8) != 0;
        g.dpad_right = (b & 1 << 9) != 0;
        g.back = (b & 1 << 10) != 0;
        g.start = (b & 1 << 11) != 0;
        g.guide = (b & 1 << 12) != 0;
        g.left_stick_button = (b & 1 << 13) != 0;
        g.right_stick_button = (b & 1 << 14) != 0;
        return true;
    }

    private static float clamp(float value, float min) {
        return Math.max(min, Math.min(1f, value));
    }

    /** "SOTMConfig.RELEASE_LATENCY_MS", "60" -> sets it and returns "SOTMConfig.RELEASE_LATENCY_MS 60.0". Only decode.configs statics. */
    public static String setConfig(String classDotField, String value) throws ReflectiveOperationException {
        int dot = classDotField.lastIndexOf('.');
        String className = classDotField.substring(0, dot);
        if (className.contains(".")) {
            throw new IllegalArgumentException("give the bare config class name");
        }
        // SimHardware.* (the sim's own plant constants) lives in the sim package, not decode.configs
        Class<?> owner = className.equals("SimHardware") ? SimHardware.class : Class.forName(CONFIG_PACKAGE + className);
        Field field = owner.getField(classDotField.substring(dot + 1));
        if (!Modifier.isStatic(field.getModifiers()) || Modifier.isFinal(field.getModifiers())) {
            throw new IllegalArgumentException("not a tunable static");
        }
        Class<?> type = field.getType();
        if (type == double.class) {
            field.setDouble(null, Double.parseDouble(value));
        } else if (type == int.class) {
            field.setInt(null, (int) Math.round(Double.parseDouble(value)));
        } else if (type == boolean.class) {
            field.setBoolean(null, value.equals("true") || value.equals("1"));
        } else {
            throw new IllegalArgumentException("not a number or boolean");
        }
        return classDotField + " " + field.get(null);
    }

    /** One frame as JSON: t, true pose x/y/th, odometry ox/oy/oth (null until known), held count, hive tilts, moved balls [id,x,y,z], partner r2 (omitted if none). */
    static String frameLine(double seconds, double[] pose, double[] odo, double[] shooter, int held, double[] hive, List<double[]> moved, double[] partner) {
        StringBuilder s = new StringBuilder("#frame {\"t\":").append(num(seconds))
                .append(",\"x\":").append(num(pose[0])).append(",\"y\":").append(num(pose[1])).append(",\"th\":").append(num(pose[2]))
                .append(",\"ox\":").append(num(odo[0])).append(",\"oy\":").append(num(odo[1])).append(",\"oth\":").append(num(odo[2]))
                .append(",\"turretDeg\":").append(num(shooter[0])).append(",\"hood\":").append(num(shooter[1])).append(",\"rps\":").append(num(shooter[2]))
                .append(",\"held\":").append(held)
                .append(",\"hive\":").append(arr(hive))
                .append(",\"balls\":[");
        for (int i = 0; i < moved.size(); i++) {
            double[] b = moved.get(i);
            s.append(i == 0 ? "" : ",").append('[').append((int) b[0]).append(',').append(num(b[1])).append(',').append(num(b[2])).append(',').append(num(b[3])).append(']');
        }
        s.append(']');
        if (partner != null) {
            s.append(",\"r2\":").append(arr(partner));
        }
        return s.append('}').toString();
    }

    private static String arr(double[] v) {
        StringBuilder s = new StringBuilder("[");
        for (int i = 0; i < v.length; i++) {
            s.append(i == 0 ? "" : ",").append(num(v[i]));
        }
        return s.append(']').toString();
    }

    private static String num(double v) {
        return Double.isFinite(v) ? String.format(Locale.ROOT, "%.4f", v) : "null";
    }

    static String jsonString(String text) {
        StringBuilder s = new StringBuilder("\"");
        for (char c : text.toCharArray()) {
            if (c == '"' || c == '\\') {
                s.append('\\').append(c);
            } else if (c < 0x20) {
                s.append(String.format(Locale.ROOT, "\\u%04x", (int) c));
            } else {
                s.append(c);
            }
        }
        return s.append('"').toString();
    }
}
