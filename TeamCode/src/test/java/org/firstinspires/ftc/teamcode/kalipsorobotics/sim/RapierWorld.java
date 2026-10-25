package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The real sim world: the robot and the field in Rapier, which runs in a Node process
 * (sim/physics/server.mjs). This class is the Java end of that conversation.
 *
 * Time is passed on lazily. The clock moves forward about 600,000 times in a match, mostly by tiny
 * steps, so Node is only asked to catch up when something needs the answer: a motor power changes
 * (the old power must be played up to now first) or a sensor is read.
 *
 * The field is built from FieldConfig, and the robot's body from RobotFootprint, the same numbers the
 * auto's path planner uses: the walls, the four Flowers, the two Hive legs. If those are wrong the
 * sim is wrong in the same way, and measuring them once fixes both. UNMEASURED: the robot mass.
 */
public final class RapierWorld implements SimWorld {

    private static final String[] MOTOR_NAMES = {"fLeft", "fRight", "bLeft", "bRight"};
    private static final int FL = 0, FR = 1, BL = 2, BR = 3;
    static final double FIELD_SIZE_MM = 3657.6;
    private static final double ROBOT_MASS_KG = 12; // UNMEASURED

    private final Process node;
    private final BufferedWriter toNode;
    private final BufferedReader fromNode;

    private final double[] power = new double[4];
    private final boolean[] brake = {true, true, true, true};
    private double x, y, theta;
    private double leftPodMm, rightPodMm, backPodMm, fLeftWheelMm;
    private long targetNanos;
    private long syncedNanos;
    private boolean initialized;
    private double[] hiveAngles = {0, 0};
    private final java.util.Map<String, java.util.function.DoubleSupplier> actuators = new java.util.HashMap<>();
    private int held;
    private String slots = "0000";
    private final List<String> events = new ArrayList<>();
    private final List<double[]> ballInfo = new ArrayList<>();
    private final List<String> warnings = new ArrayList<>();

    public RapierWorld() throws IOException {
        File nodeBinary = new File("../sim/.node/bin/node").getAbsoluteFile();
        File logFile = new File("build/sim-runs/physics.log");
        logFile.getParentFile().mkdirs();
        node = new ProcessBuilder(nodeBinary.exists() ? nodeBinary.getPath() : "node", "server.mjs")
                .directory(new File("../sim/physics").getAbsoluteFile())
                .redirectError(ProcessBuilder.Redirect.appendTo(logFile))
                .start();
        toNode = new BufferedWriter(new OutputStreamWriter(node.getOutputStream()));
        fromNode = new BufferedReader(new InputStreamReader(node.getInputStream()));
    }

    // ---- time and sensors ----

    /** Only notes the time. Node catches up when the answer is needed. */
    @Override
    public synchronized void advanceTo(long toNanos) {
        targetNanos = toNanos;
    }

    @Override
    public synchronized void setPose(double xMm, double yMm, double thetaRad) {
        x = xMm;
        y = yMm;
        theta = thetaRad;
        initialized = false; // the next sync starts the world from this pose
    }

    @Override
    public synchronized double getX() {
        syncToNow();
        return x;
    }

    @Override
    public synchronized double getY() {
        syncToNow();
        return y;
    }

    @Override
    public synchronized double getTheta() {
        syncToNow();
        return theta;
    }

    @Override
    public synchronized void setMotorPower(String motorName, double motorPower) {
        syncToNow(); // play the old power up to this moment first
        power[indexOf(motorName)] = motorPower;
    }

    @Override
    public synchronized void setBrake(String motorName, boolean brakeWhenZeroPower) {
        syncToNow();
        brake[indexOf(motorName)] = brakeWhenZeroPower;
    }

    @Override
    public synchronized double rawEncoderTicks(String motorName) {
        syncToNow();
        switch (indexOf(motorName)) {
            case BL:
                return leftPodMm / KinematicWorld.TRUE_MM_PER_TICK;
            case BR:
                return -rightPodMm / KinematicWorld.TRUE_MM_PER_TICK;
            case FR:
                return backPodMm / KinematicWorld.TRUE_MM_PER_TICK;
            default:
                return fLeftWheelMm / KinematicWorld.TRUE_MM_PER_TICK;
        }
    }

    @Override
    public synchronized double rawYawDegrees() {
        syncToNow();
        return -Math.toDegrees(theta);
    }

    @Override
    public synchronized void setActuator(String name, java.util.function.DoubleSupplier value) {
        actuators.put(name, value);
    }

    @Override
    public synchronized boolean beamBlocked(int position) {
        syncToNow();
        return slots.charAt(position) == '1';
    }

    @Override
    public synchronized int heldBalls() {
        syncToNow();
        return held;
    }

    private double actuator(String name, double fallback) {
        java.util.function.DoubleSupplier supplier = actuators.get(name);
        return supplier == null ? fallback : supplier.getAsDouble();
    }

    @Override
    public synchronized List<double[]> ballInfo() {
        syncToNow();
        return new ArrayList<>(ballInfo);
    }

    @Override
    public synchronized List<double[]> ballsMoved() {
        syncToNow();
        String[] reply = request("balls").split(" ");
        hiveAngles = new double[]{Double.parseDouble(reply[0]), Double.parseDouble(reply[1])};
        List<double[]> moved = new ArrayList<>();
        if (!reply[2].equals("-")) {
            for (String ball : reply[2].split(";")) {
                String[] v = ball.split(",");
                moved.add(new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2]), Double.parseDouble(v[3])});
            }
        }
        return moved;
    }

    @Override
    public synchronized double[] hiveAngles() {
        return hiveAngles.clone();
    }

    @Override
    public synchronized List<String> drainEvents() {
        List<String> drained = new ArrayList<>(events);
        events.clear();
        return drained;
    }

    @Override
    public synchronized List<String> warnings() {
        return new ArrayList<>(warnings);
    }

    @Override
    public synchronized void close() {
        try {
            toNode.write("quit\n");
            toNode.flush();
        } catch (IOException ignored) {
            // Node may already be gone
        }
        node.destroy();
    }

    // ---- the conversation with Node ----

    private void syncToNow() {
        if (!initialized) {
            sendInit();
        }
        if (syncedNanos == targetNanos) {
            return;
        }
        StringBuilder line = new StringBuilder("step ").append(targetNanos);
        for (double p : power) {
            line.append(' ').append(p);
        }
        for (boolean b : brake) {
            line.append(' ').append(b ? 1 : 0);
        }
        line.append(' ').append(actuator("intake", 0)).append(' ').append(actuator("stopper", 0.47)).append(' ').append(actuator("flywheelRps", 0))
                .append(' ').append(actuator("hood", 0)).append(' ').append(actuator("turretRad", 0));
        String[] v = request(line.toString()).split(" ");
        x = Double.parseDouble(v[0]);
        y = Double.parseDouble(v[1]);
        theta = Double.parseDouble(v[2]);
        leftPodMm = Double.parseDouble(v[3]);
        rightPodMm = Double.parseDouble(v[4]);
        backPodMm = Double.parseDouble(v[5]);
        fLeftWheelMm = Double.parseDouble(v[6]);
        held = Integer.parseInt(v[7]);
        slots = v[8];
        if (!v[9].equals("-")) {
            for (String event : v[9].split(";")) { // name@nanos
                int at = event.lastIndexOf('@');
                String what = event.substring(0, at).replace('~', ' ');
                // "*text" is a sentence as it is, "red-hive tipped" is a Hive event, anything else is something the robot touched
                String text = what.startsWith("*") ? what.substring(1) : what.endsWith(" tipped") ? what : "robot touched " + what;
                events.add(String.format(Locale.ROOT, "%.3f|%s", Long.parseLong(event.substring(at + 1)) / 1e9, text));
            }
        }
        syncedNanos = targetNanos;
    }

    private void sendInit() {
        List<String> names = obstacleNames();
        List<double[]> bounds = obstacleBounds();

        StringBuilder boxJson = new StringBuilder();
        for (int i = 0; i < names.size(); i++) {
            double[] b = bounds.get(i);
            boxJson.append(i == 0 ? "" : ",").append(String.format(Locale.ROOT,
                    "{\"name\":\"%s\",\"cx\":%s,\"cy\":%s,\"hw\":%s,\"hh\":%s}",
                    names.get(i), (b[0] + b[2]) / 2, (b[1] + b[3]) / 2, (b[2] - b[0]) / 2, (b[3] - b[1]) / 2));
        }
        checkStartPose(names, bounds);

        String json = String.format(Locale.ROOT,
                "{\"pose\":[%s,%s,%s],"
                        + "\"robot\":{\"frontMm\":%s,\"backMm\":%s,\"widthMm\":%s,\"massKg\":%s},"
                        + "\"field\":{\"sizeMm\":%s,\"boxes\":[%s]},"
                        + "\"drive\":{\"kV\":%s,\"kOmegaMmPerRad\":%s,\"responseSeconds\":%s,\"brakeResponseSeconds\":%s,"
                        + "\"trackWidthMm\":%s,\"backDistanceMm\":%s},"
                        + "\"balls\":%s}",
                x, y, theta,
                RobotFootprint.FRONT_MM, RobotFootprint.BACK_MM, RobotFootprint.WIDTH_MM, ROBOT_MASS_KG,
                FIELD_SIZE_MM, boxJson,
                KinematicWorld.K_V, KinematicWorld.K_OMEGA_MM_PER_RAD, KinematicWorld.WHEEL_RESPONSE_SECONDS,
                KinematicWorld.WHEEL_BRAKE_RESPONSE_SECONDS, KinematicWorld.TRUE_TRACK_WIDTH_MM, KinematicWorld.TRUE_BACK_DISTANCE_MM,
                ballsJson());
        String reply = request("init " + json);
        if (!reply.startsWith("ok")) {
            throw new IllegalStateException("physics server did not start: " + reply);
        }
        ballInfo.clear();
        if (reply.length() > 3) {
            for (String ball : reply.substring(3).split(";")) {
                String[] v = ball.split(",");
                ballInfo.add(new double[]{Double.parseDouble(v[0]), Double.parseDouble(v[1]), Double.parseDouble(v[2])});
            }
        }
        initialized = true;
        syncedNanos = 0;
        leftPodMm = rightPodMm = backPodMm = fLeftWheelMm = 0;
    }

    /**
     * A start pose with the body already inside a wall or obstacle cannot be real. Rapier would push
     * the robot out in the first milliseconds, so the code (which believes it is at the start pose)
     * would be wrong by that distance for the whole run. This says so instead of hiding it.
     */
    private void checkStartPose(List<String> names, List<double[]> bounds) {
        warnings.clear();
        Point centre = new Point(x, y);
        double outside = 0;
        for (double[] corner : RobotFootprint.corners(centre, theta)) {
            outside = Math.max(outside, Math.max(Math.max(-corner[0], corner[0] - FIELD_SIZE_MM), Math.max(-corner[1], corner[1] - FIELD_SIZE_MM)));
        }
        if (outside > 0.5) {
            warnings.add(String.format(Locale.ROOT,
                    "The start pose puts the robot body %.0f mm outside the field wall. The robot is pushed out in the first "
                            + "milliseconds, so the code is that far off from the start. Measure RobotFootprint, or fix the start pose.", outside));
        }
        for (int i = 0; i < names.size(); i++) {
            if (RobotFootprint.overlaps(centre, theta, bounds.get(i), 0)) {
                warnings.add("The start pose puts the robot body inside " + names.get(i) + ".");
            }
        }
    }

    /**
     * Where the manual puts the balls (Figure 10-2), in the Red frame: the four Flowers and the two Gardens from FieldConfig, and the
     * two Hives with 3 nectar in the up Cell of each. The red Hive's up Cell is the +y one, the blue Hive's is the -y one.
     */
    private static String ballsJson() {
        // Every -Dsim.physics.<name>=<number> goes to the physics as a ball parameter, so adding one needs no Java change.
        StringBuilder params = new StringBuilder();
        int preload = 4; // the manual: exactly 4 pollen
        for (String key : System.getProperties().stringPropertyNames()) {
            if (key.startsWith("sim.physics.")) {
                String name = key.substring("sim.physics.".length());
                if (name.equals("preload")) {
                    preload = (int) Double.parseDouble(System.getProperty(key));
                } else {
                    params.append(params.length() == 0 ? "" : ",").append('"').append(name).append("\":").append(Double.parseDouble(System.getProperty(key)));
                }
            }
        }
        double pollenRadius = 35.56;
        double hiveCentreX = (FieldConfig.HIVE_MIN_X + FieldConfig.HIVE_MAX_X) / 2;
        double hiveCentreY = (FieldConfig.HIVE_MIN_Y + FieldConfig.HIVE_MAX_Y) / 2;
        double hiveOffset = 25.5 * 25.4 / 2; // the two Hives are 25.5 in apart (Figure 9-10)
        Point[] flowers = {FieldConfig.aFlowerPoint, FieldConfig.bFlowerPoint, FieldConfig.cFlowerPoint, FieldConfig.dFlowerPoint};
        String[] names = {"A", "B", "C", "D"};
        StringBuilder flowerJson = new StringBuilder();
        for (int i = 0; i < flowers.length; i++) {
            flowerJson.append(i == 0 ? "" : ",").append(String.format(Locale.ROOT, "{\"name\":\"%s\",\"x\":%s,\"y\":%s}", names[i], flowers[i].getX(), flowers[i].getY()));
        }
        return String.format(Locale.ROOT,
                "{\"flowers\":[%s],"
                        + "\"gardens\":[{\"name\":\"red\",\"x0\":%s,\"y0\":%s,\"dx\":%s,\"dy\":0},{\"name\":\"blue\",\"x0\":%s,\"y0\":%s,\"dx\":%s,\"dy\":0}],"
                        + "\"preload\":%d,\"params\":{%s},"
                        + "\"hives\":[{\"name\":\"red-hive\",\"x\":%s,\"y\":%s,\"upSide\":1,\"nectarType\":1},{\"name\":\"blue-hive\",\"x\":%s,\"y\":%s,\"upSide\":-1,\"nectarType\":2}]}",
                flowerJson,
                pollenRadius, FIELD_SIZE_MM - pollenRadius, 2 * pollenRadius,
                FIELD_SIZE_MM - pollenRadius, pollenRadius, -2 * pollenRadius,
                preload, params,
                hiveCentreX - hiveOffset, hiveCentreY, hiveCentreX + hiveOffset, hiveCentreY);
    }

    /** What the robot can touch besides the walls. Same order as obstacleBounds(). */
    static List<String> obstacleNames() {
        List<String> names = new ArrayList<>();
        names.add("flower-A");
        names.add("flower-B");
        names.add("flower-C");
        names.add("flower-D");
        names.add("hive-leg-low");
        names.add("hive-leg-high");
        return names;
    }

    /** {minX, minY, maxX, maxY} in mm for each obstacleNames() entry, straight from FieldConfig. */
    static List<double[]> obstacleBounds() {
        double flowerHalf = FieldConfig.FLOWER_SIZE_MM / 2;
        List<double[]> bounds = new ArrayList<>();
        for (Point flower : new Point[]{FieldConfig.aFlowerPoint, FieldConfig.bFlowerPoint, FieldConfig.cFlowerPoint, FieldConfig.dFlowerPoint}) {
            bounds.add(new double[]{flower.getX() - flowerHalf, flower.getY() - flowerHalf, flower.getX() + flowerHalf, flower.getY() + flowerHalf});
        }
        bounds.add(FieldConfig.HIVE_LEG_LOW_X);
        bounds.add(FieldConfig.HIVE_LEG_HIGH_X);
        return bounds;
    }

    private String request(String line) {
        try {
            toNode.write(line);
            toNode.write('\n');
            toNode.flush();
            String reply = fromNode.readLine();
            if (reply == null) {
                throw new IllegalStateException("the physics server stopped; see build/sim-runs/physics.log");
            }
            if (reply.startsWith("error")) {
                throw new IllegalStateException("physics server: " + reply);
            }
            return reply;
        } catch (IOException e) {
            throw new IllegalStateException("could not talk to the physics server", e);
        }
    }

    private static int indexOf(String motorName) {
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            if (MOTOR_NAMES[i].equals(motorName)) {
                return i;
            }
        }
        throw new IllegalArgumentException("RapierWorld does not know a drive motor named \"" + motorName + "\"");
    }
}
