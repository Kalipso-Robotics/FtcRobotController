package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.URLEncoder;
import java.util.List;
import java.util.Map;

/**
 * One sim run in its own JVM: runs an OpMode and writes what it recorded to a file.
 * Started by SimRunner.runInNewJvm (and later by the browser's server).
 *
 * Arguments: opModeClassName startX startY startThetaRad outFile [rapier|kinematic]
 * File: lines "#used 30.0", "#finished true", "#matchStart 3.0", "#event 7.4|robot touched wall-x0",
 * "#balltype <id> <type> <radius>", "#ball <seconds> <id> <x> <y> <z>" (every move), "#hivetilt <seconds> <red> <blue>", "#held <seconds> <count>", "#assumed <text>", "#warning <text>", "#param <id> <value used> <default>", "#field/#robot/#obstacle/#point lines (the geometry the UI draws, from FieldConfig and
 * RobotFootprint), "#telemetry <seconds> key=value&key=value" (URL-encoded), "#error <text, one line per line starting with #error>",
 * then one CSV line per frame: seconds,trueX,trueY,trueTheta,odoX,odoY,odoTheta
 */
public final class SimMain {

    private SimMain() {
    }

    public static void main(String[] args) throws Exception {
        if (args.length > 0 && args[0].equals("--defaults")) {
            // "#param <id> <default>" for every parameter, for the Setup screen
            for (Map.Entry<String, Double> entry : SimParams.defaults().entrySet()) {
                System.out.println("#param " + entry.getKey() + " " + entry.getValue());
            }
            System.exit(0);
        }
        int exitCode = 0;
        try {
            Map<String, Double> defaults = SimParams.defaults();
            Map<String, Double> used = SimParams.applyFromSystemProperties();
            SimRunner.Result result = SimRunner.run(args[0], Double.parseDouble(args[1]), Double.parseDouble(args[2]),
                    Double.parseDouble(args[3]), args.length > 5 ? args[5] : SimRunner.RAPIER);
            write(result, new File(args[4]), defaults, used);
        } catch (Throwable t) {
            t.printStackTrace();
            exitCode = 1;
        }
        System.exit(exitCode); // the robot code's executor threads are not daemons
    }

    private static void write(SimRunner.Result result, File outFile, Map<String, Double> defaults, Map<String, Double> used) throws IOException {
        try (PrintWriter out = new PrintWriter(outFile)) {
            out.println("#used " + result.virtualSecondsUsed);
            out.println("#finished " + result.finishedInTime);
            out.println("#matchStart " + result.matchStartSeconds);
            for (String event : result.events) {
                out.println("#event " + event);
            }
            for (String warning : result.warnings) {
                out.println("#warning " + warning);
            }
            writeGeometry(out);
            if (result.partnerInfo != null) {
                out.println("#partnerinfo " + result.partnerInfo);
            }
            for (double[] info : result.ballInfo) {
                out.println("#balltype " + (int) info[0] + " " + (int) info[1] + " " + info[2]);
            }
            for (double[] move : result.ballMoves) {
                out.println("#ball " + move[0] + " " + (int) move[1] + " " + (int) move[2] + " " + (int) move[3] + " " + (int) move[4]);
            }
            for (double[] p : result.partnerFrames) {
                out.println("#partner " + p[0] + " " + p[1] + " " + p[2] + " " + p[3]);
            }
            for (double[] held : result.heldFrames) {
                out.println("#held " + held[0] + " " + (int) held[1]);
            }
            for (String call : result.assumed) {
                out.println("#assumed " + call);
            }
            for (double[] hive : result.hiveFrames) {
                out.println("#hivetilt " + hive[0] + " " + String.format(java.util.Locale.ROOT, "%.4f %.4f", hive[1], hive[2]));
            }
            for (Map.Entry<String, Double> entry : used.entrySet()) {
                out.println("#param " + entry.getKey() + " " + entry.getValue() + " " + defaults.get(entry.getKey())); // id used default
            }
            for (Map<String, String> update : result.telemetryUpdates) {
                StringBuilder line = new StringBuilder("#telemetry ").append(update.get("_t")).append(' ');
                boolean first = true;
                for (Map.Entry<String, String> entry : update.entrySet()) {
                    if (entry.getKey().equals("_t")) {
                        continue;
                    }
                    line.append(first ? "" : "&").append(encode(entry.getKey())).append('=').append(encode(entry.getValue()));
                    first = false;
                }
                out.println(line);
            }
            if (result.error != null) {
                for (String line : result.error.split("\n")) {
                    out.println("#error " + line);
                }
            }
            for (double[] f : result.frames) {
                StringBuilder row = new StringBuilder();
                for (int i = 0; i < f.length; i++) {
                    row.append(i == 0 ? "" : ",").append(f[i]);
                }
                out.println(row);
            }
        }
    }

    /** What the browser draws. The same FieldConfig and RobotFootprint numbers the auto and the physics use. */
    private static void writeGeometry(PrintWriter out) {
        out.println("#field " + RapierWorld.FIELD_SIZE_MM);
        out.println("#robot " + RobotFootprint.FRONT_MM + " " + RobotFootprint.BACK_MM + " " + RobotFootprint.WIDTH_MM);
        List<String> names = RapierWorld.obstacleNames();
        List<double[]> bounds = RapierWorld.obstacleBounds();
        for (int i = 0; i < names.size(); i++) {
            double[] b = bounds.get(i);
            out.println("#obstacle " + names.get(i) + " " + b[0] + " " + b[1] + " " + b[2] + " " + b[3]);
        }
        out.println("#hive " + FieldConfig.HIVE_MIN_X + " " + FieldConfig.HIVE_MIN_Y + " " + FieldConfig.HIVE_MAX_X + " " + FieldConfig.HIVE_MAX_Y);
        out.println("#midline " + FieldConfig.MIDLINE_X);
        Object[][] points = {{"launch-A", FieldConfig.aLaunchPoint}, {"launch-B", FieldConfig.bLaunchPoint},
                {"target-A", FieldConfig.aTargetPoint}, {"target-B", FieldConfig.bTargetPoint}, {"garden", FieldConfig.gardenPoint}};
        for (Object[] point : points) {
            out.println("#point " + point[0] + " " + ((Point) point[1]).getX() + " " + ((Point) point[1]).getY());
        }
    }

    private static String encode(String text) throws IOException {
        return URLEncoder.encode(text, "UTF-8");
    }

    static SimRunner.Result readResult(File file) throws IOException {
        SimRunner.Result result = new SimRunner.Result();
        StringBuilder error = new StringBuilder();
        try (BufferedReader in = new BufferedReader(new FileReader(file))) {
            for (String line = in.readLine(); line != null; line = in.readLine()) {
                if (line.startsWith("#used ")) {
                    result.virtualSecondsUsed = Double.parseDouble(line.substring(6));
                } else if (line.startsWith("#matchStart ")) {
                    result.matchStartSeconds = Double.parseDouble(line.substring(12));
                } else if (line.startsWith("#event ")) {
                    result.events.add(line.substring(7));
                } else if (line.startsWith("#warning ")) {
                    result.warnings.add(line.substring(9));
                } else if (line.startsWith("#finished ")) {
                    result.finishedInTime = Boolean.parseBoolean(line.substring(10));
                } else if (line.startsWith("#error ")) {
                    error.append(line.substring(7)).append('\n');
                } else if (line.startsWith("#balltype ")) {
                    String[] v = line.substring(10).split(" ");
                    result.ballInfo.add(new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2])});
                } else if (line.startsWith("#ball ")) {
                    String[] v = line.substring(6).split(" ");
                    result.ballMoves.add(new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2]), Double.parseDouble(v[3]), Double.parseDouble(v[4])});
                } else if (line.startsWith("#hivetilt ")) {
                    String[] v = line.substring(10).split(" ");
                    result.hiveFrames.add(new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2])});
                } else if (line.startsWith("#held ")) {
                    String[] v = line.substring(6).split(" ");
                    result.heldFrames.add(new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1])});
                } else if (line.startsWith("#partnerinfo ")) {
                    result.partnerInfo = line.substring(13);
                } else if (line.startsWith("#partner ")) {
                    String[] v = line.substring(9).split(" ");
                    result.partnerFrames.add(new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2]), Double.parseDouble(v[3])});
                } else if (line.startsWith("#assumed ")) {
                    result.assumed.add(line.substring(9));
                } else if (line.startsWith("#")) {
                    continue; // geometry and telemetry are for the browser
                } else if (!line.isEmpty()) {
                    String[] parts = line.split(",");
                    double[] frame = new double[parts.length];
                    for (int i = 0; i < parts.length; i++) {
                        frame[i] = Double.parseDouble(parts[i]);
                    }
                    result.frames.add(frame);
                }
            }
        }
        if (error.length() > 0) {
            result.error = error.toString();
        }
        return result;
    }
}
