package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

import java.io.File;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Runs one of the team's real @Autonomous OpModes on the laptop, on a virtual clock, against the
 * simulated robot. The OpMode class is the compiled robot code, not a copy.
 *
 * What it does, in order: loads the OpMode class (after the clock rewrite is in place), gives it the
 * fake hardwareMap, runs runOpMode() on its own thread, "presses START" after the init time, and
 * stops it 30 virtual seconds later, the way the field does.
 */
public final class SimRunner {

    // UNMEASURED: how long one loop takes on the real robot. These decide how often the code reads
    // sensors and updates motors, so they matter. Measure from a real log (OdometryLogger).
    private static final long MAIN_LOOP_NANOS = 10_000_000L;
    private static final long BACKGROUND_LOOP_NANOS = 2_000_000L;
    private static final long YIELD_NANOS = 50_000L;

    private static final double INIT_SECONDS = 3;
    private static final double AUTO_SECONDS = 30;
    private static final long FRAME_NANOS = 20_000_000L;
    private static final long REAL_TIMEOUT_MS = 180_000;

    /** What a run produced. */
    public static final class Result {
        /** One row per frame: virtual seconds, true x, y, theta, then the odometry's x, y, theta (NaN if not yet known). */
        public final List<double[]> frames = new ArrayList<>();
        public final List<Map<String, String>> telemetryUpdates = new ArrayList<>();
        /** What each ball is: {id, type (0 pollen, 1 red nectar, 2 blue nectar), radius mm}. */
        public final List<double[]> ballInfo = new ArrayList<>();
        /** Every time a ball moved: {seconds, id, x, y, z} in mm. A ball that sits still is not repeated. */
        public final List<double[]> ballMoves = new ArrayList<>();
        /** How many balls the robot holds at each frame: {seconds, count}. */
        public final List<double[]> heldFrames = new ArrayList<>();
        /** Hardware calls the sim does not model, which were assumed to work. */
        public final List<String> assumed = new ArrayList<>();
        /** The red and blue Hive tilt at each frame: {seconds, red, blue} in radians. */
        public final List<double[]> hiveFrames = new ArrayList<>();
        /** Things that happened, as "seconds|text" in virtual seconds (the match starts at matchStartSeconds). */
        public final List<String> events = new ArrayList<>();
        /** Problems with the setup that make this run less trustworthy. */
        public final List<String> warnings = new ArrayList<>();
        public double matchStartSeconds;
        /** The stack trace if the OpMode threw, otherwise null. */
        public String error;
        public double virtualSecondsUsed;
        public boolean finishedInTime;
    }

    private SimRunner() {
    }

    /** The two worlds. Rapier is the real one; kinematic is the open-field cross-check. */
    public static final String RAPIER = "rapier";
    public static final String KINEMATIC = "kinematic";

    /**
     * @param startX,startY,startTheta where the robot really starts (mm, mm, rad). It must match the start the
     *                                 OpMode gives its Odometry, or the code will believe it is somewhere else.
     * @param worldKind                RAPIER or KINEMATIC
     */
    public static Result run(String opModeClassName, double startX, double startY, double startTheta, String worldKind) throws Exception {
        ClockRewriter.install();

        SimHardware.clearAssumed();
        Result result = new Result();
        SimWorld world = worldKind.equals(KINEMATIC) ? new KinematicWorld() : new RapierWorld();
        world.setPose(startX, startY, startTheta);

        long[] nextFrame = {0};
        long[] matchEndNanos = {Long.MAX_VALUE};
        SimClock.start(toNanos -> {
            world.advanceTo(toNanos);
            if (toNanos >= nextFrame[0]) {
                result.frames.add(frame(toNanos, world));
                recordBalls(result, world, toNanos);
                nextFrame[0] = toNanos - toNanos % FRAME_NANOS + FRAME_NANOS;
            }
        }, BACKGROUND_LOOP_NANOS, YIELD_NANOS);

        try {
            LinearOpMode opMode = (LinearOpMode) Class.forName(opModeClassName).getDeclaredConstructor().newInstance();
            opMode.hardwareMap = SimHardware.createHardwareMap(world);
            opMode.telemetry = SimHardware.createTelemetry(result.telemetryUpdates);
            opMode.gamepad1 = new Gamepad();
            opMode.gamepad2 = new Gamepad();

            SimClock.Participant opModeSlot = SimClock.create("opmode", MAIN_LOOP_NANOS);
            SimClock.Participant refereeSlot = SimClock.create("referee", MAIN_LOOP_NANOS);

            Thread opModeThread = new Thread(() -> {
                opModeSlot.bindToCurrentThread();
                try {
                    opMode.runOpMode();
                } catch (Throwable t) {
                    if (!(t instanceof InterruptedException)) {
                        result.error = stackTrace(t);
                    }
                } finally {
                    SimClock.threadFinished();
                }
            }, "sim-opmode");

            Thread refereeThread = new Thread(() -> {
                refereeSlot.bindToCurrentThread();
                try {
                    SimClock.sleep((long) (INIT_SECONDS * 1000));
                    setFlag(opMode, "isStarted", true);
                    opMode.resetRuntime();
                    long startedAt = SimClock.elapsedNanos();
                    result.matchStartSeconds = startedAt / 1e9;
                    SimClock.sleep((long) (AUTO_SECONDS * 1000));
                    matchEndNanos[0] = SimClock.elapsedNanos();
                    result.frames.add(frame(matchEndNanos[0], world));
                    recordBalls(result, world, matchEndNanos[0]);
                    setFlag(opMode, "stopRequested", true);
                    opModeThread.interrupt(); // the field stops a LinearOpMode by interrupting its thread
                    result.virtualSecondsUsed = (SimClock.elapsedNanos() - startedAt) / 1e9;
                } catch (Throwable t) {
                    result.error = stackTrace(t);
                } finally {
                    SimClock.threadFinished();
                }
            }, "sim-referee");

            opModeThread.start();
            refereeThread.start();
            SimClock.go();

            opModeThread.join(REAL_TIMEOUT_MS);
            result.finishedInTime = !opModeThread.isAlive();
            refereeThread.join(1000);
            // The OpMode needs a moment of virtual time to unwind after STOP. That is not part of the match.
            result.frames.removeIf(f -> f[0] > matchEndNanos[0] / 1e9);
            result.ballMoves.removeIf(m -> m[0] > matchEndNanos[0] / 1e9);
            result.hiveFrames.removeIf(m -> m[0] > matchEndNanos[0] / 1e9);
            result.heldFrames.removeIf(m -> m[0] > matchEndNanos[0] / 1e9);
            result.assumed.addAll(SimHardware.assumedCalls());
            for (String event : world.drainEvents()) {
                if (Double.parseDouble(event.substring(0, event.indexOf('|'))) <= matchEndNanos[0] / 1e9) {
                    result.events.add(event);
                }
            }
            result.warnings.addAll(world.warnings());
        } finally {
            SimClock.stop();
            world.close();
        }
        return result;
    }

    /**
     * Runs the OpMode in a NEW JVM and reads back what it recorded. Use this, not run(): the robot code
     * keeps static state (SharedData, the Odometry and DriveTrain singletons), so a second run in the
     * same JVM starts from the first run's leftovers and gives a different result.
     */
    public static Result runInNewJvm(String opModeClassName, double startX, double startY, double startTheta) throws Exception {
        return runInNewJvm(opModeClassName, startX, startY, startTheta, RAPIER);
    }

    public static Result runInNewJvm(String opModeClassName, double startX, double startY, double startTheta, String worldKind) throws Exception {
        File outFile = new File("build/sim-runs/" + opModeClassName.substring(opModeClassName.lastIndexOf('.') + 1) + "-" + System.nanoTime() + ".csv");
        outFile.getParentFile().mkdirs();
        File logFile = new File(outFile.getPath() + ".log");

        String java = System.getProperty("java.home") + File.separator + "bin" + File.separator + "java";
        Process process = new ProcessBuilder(java, "-Djdk.attach.allowAttachSelf=true",
                "-cp", System.getProperty("sim.classpath"), SimMain.class.getName(),
                opModeClassName, String.valueOf(startX), String.valueOf(startY), String.valueOf(startTheta), outFile.getPath(), worldKind)
                .redirectErrorStream(true).redirectOutput(logFile).start();
        if (!process.waitFor(REAL_TIMEOUT_MS + 30_000, TimeUnit.MILLISECONDS)) {
            process.destroyForcibly();
            throw new IllegalStateException("the sim JVM did not finish; see " + logFile);
        }
        if (!outFile.exists()) {
            throw new IllegalStateException("the sim JVM wrote no result (exit " + process.exitValue() + "); see " + logFile);
        }
        return SimMain.readResult(outFile);
    }

    /** The balls that moved since the last frame, and the Hive tilt. The first call reports every ball (where they settled). */
    private static void recordBalls(Result result, SimWorld world, long nanos) {
        if (result.ballInfo.isEmpty()) {
            result.ballInfo.addAll(world.ballInfo());
        }
        if (result.ballInfo.isEmpty()) {
            return; // a world with no balls
        }
        double seconds = nanos / 1e9;
        for (double[] ball : world.ballsMoved()) {
            result.ballMoves.add(new double[]{seconds, ball[0], ball[1], ball[2], ball[3]});
        }
        double[] hive = world.hiveAngles();
        result.hiveFrames.add(new double[]{seconds, hive[0], hive[1]});
        result.heldFrames.add(new double[]{seconds, world.heldBalls()});
    }

    static String stackTrace(Throwable t) {
        StringWriter text = new StringWriter();
        t.printStackTrace(new PrintWriter(text));
        return text.toString();
    }

    private static double[] frame(long nanos, SimWorld world) {
        double odoX = Double.NaN, odoY = Double.NaN, odoTheta = Double.NaN;
        Position odo = SharedData.getOdometryWheelIMUPosition();
        if (odo != null) {
            odoX = odo.getX();
            odoY = odo.getY();
            odoTheta = odo.getTheta();
        }
        return new double[]{nanos / 1e9, world.getX(), world.getY(), world.getTheta(), odoX, odoY, odoTheta};
    }

    private static void setFlag(LinearOpMode opMode, String fieldName, boolean value) throws ReflectiveOperationException {
        Field field = Class.forName("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal").getDeclaredField(fieldName);
        field.setAccessible(true);
        field.setBoolean(opMode, value);
    }
}
