package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import android.content.Context;
import android.content.ContextWrapper;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.io.File;
import java.lang.reflect.Proxy;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The fake FTC hardware the auto code gets from hardwareMap. Each device has the same methods as
 * the real one. Methods the sim does not model yet THROW instead of returning an invented value,
 * so a missing piece shows up as an error, never as a quietly wrong run.
 *
 * Names must match the robot configuration. Asking for a name that is not here throws, as on the robot.
 */
public final class SimHardware {

    private static final double SHOOTER_TICKS_PER_REV = 28;                  // CalculateTickPer: RPS = ticks per second / 28
    private static final double TURRET_TICKS_PER_RAD = 384.5 * 125 / 32 / (2 * Math.PI); // TurretConfig: 384.5 ticks per rev, 125/32 pulley
    private static final String[] DRIVE_MOTORS = {"fLeft", "fRight", "bLeft", "bRight"};

    // UNMEASURED: battery voltage. A real battery sags under load; this is a flat number for now.
    private static final double BATTERY_VOLTS = 13.0;

    private SimHardware() {
    }

    public static HardwareMap createHardwareMap(SimWorld world) {
        SimHardwareMap map = new SimHardwareMap();
        for (int port = 0; port < DRIVE_MOTORS.length; port++) {
            map.add(DRIVE_MOTORS[port], new SimMotor(DRIVE_MOTORS[port], port, world));
        }
        map.add("imu", new SimImu(world));
        map.add("Control Hub", new SimVoltageSensor());

        // The ball hardware. Shooter speed numbers come from the code's own PIDF: kF = 0.01148 means about 87 RPS at full power.
        // UNMEASURED: how fast each one spins up, and the intake and turret top speeds.
        SimAuxMotor shooter1 = new SimAuxMotor("shooter1", 0, SHOOTER_TICKS_PER_REV * 87, 0.35);
        SimAuxMotor shooter2 = new SimAuxMotor("shooter2", 1, SHOOTER_TICKS_PER_REV * 87, 0.35);
        SimAuxMotor intake = new SimAuxMotor("intake", 2, 2000, 0.08);
        SimAuxMotor turret = new SimAuxMotor("turret", 3, TURRET_TICKS_PER_RAD * 3.0, 0.05);
        SimServo hood = new SimServo("hood", 0);
        SimServo stopper = new SimServo("stopper", 1);
        for (SimDevice device : new SimDevice[]{shooter1, shooter2, intake, turret, hood, stopper, new SimServo("tilterLeft", 2), new SimServo("tilterRight", 3)}) {
            map.add(device.getDeviceName(), device);
        }
        world.setActuator("intake", intake::getPower);
        world.setActuator("flywheelRps", () -> shooter1.getVelocity() / SHOOTER_TICKS_PER_REV);
        world.setActuator("stopper", stopper::getPosition);
        world.setActuator("hood", hood::getPosition);
        world.setActuator("turretRad", () -> turret.getCurrentPosition() / TURRET_TICKS_PER_RAD);

        // The four beam breaks are analog inputs: about 3.3 V when clear, about 0 V when a ball blocks the beam.
        SimBeamBreaks beams = new SimBeamBreaks(world);
        String[] beamNames = {"beamBreakAOuter", "beamBreakAInner", "beamBreakBInner", "beamBreakBOuter"};
        for (int channel = 0; channel < beamNames.length; channel++) {
            map.add(beamNames[channel], new AnalogInput(beams, channel));
        }
        return map;
    }

    /**
     * The SDK's HardwareMap.tryGet() first asks Device.isRevControlHub(), which loads an Android-only
     * native library and cannot run on a laptop. This map answers by name instead. Everything else
     * (hardwareMap.dcMotor.get(...), servo, digitalChannel, voltageSensor) is the SDK's own code.
     */
    private static final class SimHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> byName = new LinkedHashMap<>();

        SimHardwareMap() {
            super(new SimContext(), null);
        }

        void add(String name, HardwareDevice device) {
            byName.put(name, device);
            if (device instanceof DcMotor) {
                dcMotor.put(name, (DcMotor) device);
            } else if (device instanceof Servo) {
                servo.put(name, (Servo) device);
            } else if (device instanceof DigitalChannel) {
                digitalChannel.put(name, (DigitalChannel) device);
            } else if (device instanceof VoltageSensor) {
                voltageSensor.put(name, (VoltageSensor) device);
            } else {
                put(name, device);
            }
        }

        @Override
        public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
            HardwareDevice device = byName.get(deviceName.trim());
            return classOrInterface.isInstance(device) ? classOrInterface.cast(device) : null;
        }
    }

    /** A Telemetry that keeps what the code shows, so the run can report it. Lines are recorded on update(). */
    public static Telemetry createTelemetry(List<Map<String, String>> updates) {
        Map<String, String> pending = new LinkedHashMap<>();
        return (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "addData":
                            pending.put(String.valueOf(args[0]), args.length > 1 && args[1] != null ? String.valueOf(args[1]) : "");
                            return null;
                        case "update":
                            Map<String, String> update = new LinkedHashMap<>();
                            update.put("_t", String.valueOf(SimClock.elapsedNanos() / 1e9)); // virtual seconds
                            update.putAll(pending);
                            updates.add(update);
                            pending.clear();
                            return true;
                        case "isAutoClear":
                            return true;
                        default:
                            Class<?> type = method.getReturnType();
                            if (type == boolean.class) {
                                return false;
                            } else if (type == int.class) {
                                return 0;
                            } else if (type == String.class) {
                                return "";
                            }
                            return null;
                    }
                });
    }

    /**
     * The app Context. The team's log writers ask it for a folder; everything else answers "nothing"
     * (the Android test stubs return null/0). Log files go to TeamCode/build/sim-files.
     */
    private static final class SimContext extends ContextWrapper {
        private final File filesDir = new File("build/sim-files");

        SimContext() {
            super((Context) null);
            filesDir.mkdirs();
        }

        @Override
        public File getExternalFilesDir(String type) {
            return filesDir;
        }
    }

    // Things the sim does not model yet are ASSUMED TO WORK: the call returns a harmless value and is recorded here, then listed
    // on the Stats page ("Assumed to work"), so nothing is hidden.
    private static final java.util.Set<String> ASSUMED = new java.util.LinkedHashSet<>();

    static synchronized void assume(String what) {
        ASSUMED.add(what);
    }

    public static synchronized List<String> assumedCalls() {
        return new java.util.ArrayList<>(ASSUMED);
    }

    public static synchronized void clearAssumed() {
        ASSUMED.clear();
    }

    // ---------------------------------------------------------------------------------------------

    private abstract static class SimDevice implements HardwareDevice {
        private final String name;

        SimDevice(String name) {
            this.name = name;
        }

        @Override
        public Manufacturer getManufacturer() {
            return Manufacturer.Unknown;
        }

        @Override
        public String getDeviceName() {
            return name;
        }

        @Override
        public String getConnectionInfo() {
            return "simulated";
        }

        @Override
        public int getVersion() {
            return 1;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
        }

        @Override
        public void close() {
        }
    }

    /**
     * A motor. setPower() goes to the world. getCurrentPosition() is what the encoder on that port
     * reads. For the drive motors the encoder is a dead-wheel pod, not the motor's own shaft.
     */
    public static final class SimMotor extends SimDevice implements DcMotorEx {
        private final int port;
        private final SimWorld world;
        private Direction direction = Direction.FORWARD;
        private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
        private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
        private double power;
        private double encoderOffsetTicks;

        SimMotor(String name, int port, SimWorld world) {
            super(name);
            this.port = port;
            this.world = world;
        }

        @Override
        public void setPower(double newPower) {
            power = newPower;
            world.setMotorPower(getDeviceName(), newPower);
        }

        @Override
        public double getPower() {
            return power;
        }

        @Override
        public int getCurrentPosition() {
            double sign = direction == Direction.REVERSE ? -1 : 1;
            return (int) Math.round(sign * (world.rawEncoderTicks(getDeviceName()) - encoderOffsetTicks));
        }

        @Override
        public void setMode(RunMode newMode) {
            if (newMode == RunMode.RUN_TO_POSITION) {
                assume("DcMotor RUN_TO_POSITION (the position target is not followed)");
            }
            if (newMode == RunMode.STOP_AND_RESET_ENCODER) {
                encoderOffsetTicks = world.rawEncoderTicks(getDeviceName());
            }
            mode = newMode;
        }

        @Override
        public RunMode getMode() {
            return mode;
        }

        @Override
        public void setDirection(Direction newDirection) {
            direction = newDirection;
        }

        @Override
        public Direction getDirection() {
            return direction;
        }

        @Override
        public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
            zeroPowerBehavior = behavior;
            world.setBrake(getDeviceName(), behavior == ZeroPowerBehavior.BRAKE);
        }

        @Override
        public ZeroPowerBehavior getZeroPowerBehavior() {
            return zeroPowerBehavior;
        }

        @Override
        public int getPortNumber() {
            return port;
        }

        @Override
        public void setPowerFloat() {
            zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
            world.setBrake(getDeviceName(), false);
            setPower(0);
        }

        @Override
        public boolean getPowerFloat() {
            return zeroPowerBehavior == ZeroPowerBehavior.FLOAT && power == 0;
        }

        @Override
        public MotorConfigurationType getMotorType() {
            assume("DcMotor.getMotorType");
            return null;
        }

        @Override
        public void setMotorType(MotorConfigurationType motorType) {
        }

        @Override
        public DcMotorController getController() {
            assume("DcMotor.getController");
            return null;
        }

        @Override
        public void setTargetPosition(int position) {
            assume("DcMotor.setTargetPosition");
        }

        @Override
        public int getTargetPosition() {
            assume("DcMotor.getTargetPosition");
            return 0;
        }

        @Override
        public boolean isBusy() {
            assume("DcMotor.isBusy");
            return false;
        }

        @Override
        public void setMotorEnable() {
        }

        @Override
        public void setMotorDisable() {
        }

        @Override
        public boolean isMotorEnabled() {
            return true;
        }

        @Override
        public void setVelocity(double angularRate) {
            assume("DcMotorEx.setVelocity");
        }

        @Override
        public void setVelocity(double angularRate, AngleUnit unit) {
            assume("DcMotorEx.setVelocity");
        }

        @Override
        public double getVelocity() {
            assume("DcMotorEx.getVelocity");
            return 0;
        }

        @Override
        public double getVelocity(AngleUnit unit) {
            assume("DcMotorEx.getVelocity");
            return 0;
        }

        @Override
        public void setPIDCoefficients(RunMode runMode, PIDCoefficients pidCoefficients) {
        }

        @Override
        public void setPIDFCoefficients(RunMode runMode, PIDFCoefficients pidfCoefficients) {
        }

        @Override
        public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        }

        @Override
        public void setPositionPIDFCoefficients(double p) {
        }

        @Override
        public PIDCoefficients getPIDCoefficients(RunMode runMode) {
            assume("DcMotorEx.getPIDCoefficients");
            return null;
        }

        @Override
        public PIDFCoefficients getPIDFCoefficients(RunMode runMode) {
            assume("DcMotorEx.getPIDFCoefficients");
            return null;
        }

        @Override
        public void setTargetPositionTolerance(int tolerance) {
        }

        @Override
        public int getTargetPositionTolerance() {
            assume("DcMotorEx.getTargetPositionTolerance");
            return 0;
        }

        @Override
        public double getCurrent(CurrentUnit unit) {
            assume("DcMotorEx.getCurrent");
            return 0;
        }

        @Override
        public double getCurrentAlert(CurrentUnit unit) {
            assume("DcMotorEx.getCurrentAlert");
            return 0;
        }

        @Override
        public void setCurrentAlert(double current, CurrentUnit unit) {
        }

        @Override
        public boolean isOverCurrent() {
            return false;
        }
    }

    /** The IMU. Yaw comes from the world's true heading, with resetYaw() setting zero. No drift yet (UNMEASURED). */
    public static final class SimImu extends SimDevice implements IMU {
        private final SimWorld world;
        private double yawOffsetDegrees;

        SimImu(SimWorld world) {
            super("imu");
            this.world = world;
        }

        @Override
        public boolean initialize(Parameters parameters) {
            return true;
        }

        @Override
        public void resetYaw() {
            yawOffsetDegrees = world.rawYawDegrees();
        }

        @Override
        public YawPitchRollAngles getRobotYawPitchRollAngles() {
            double yaw = AngleUnit.normalizeDegrees(world.rawYawDegrees() - yawOffsetDegrees);
            return new YawPitchRollAngles(AngleUnit.DEGREES, yaw, 0, 0, SimClock.nanoTime());
        }

        @Override
        public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
            assume("IMU.getRobotOrientation");
            return null;
        }

        @Override
        public Quaternion getRobotOrientationAsQuaternion() {
            assume("IMU.getRobotOrientationAsQuaternion");
            return null;
        }

        @Override
        public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
            assume("IMU.getRobotAngularVelocity");
            return null;
        }
    }

    public static final class SimVoltageSensor extends SimDevice implements VoltageSensor {
        SimVoltageSensor() {
            super("Control Hub");
        }

        @Override
        public double getVoltage() {
            return BATTERY_VOLTS;
        }
    }

    /** A digital input (beam break). Not wired to balls yet, so reading it throws. */
    public static final class SimDigitalChannel extends SimDevice implements DigitalChannel {
        SimDigitalChannel(String name) {
            super(name);
        }

        @Override
        public Mode getMode() {
            return Mode.INPUT;
        }

        @Override
        public void setMode(Mode mode) {
        }

        @Override
        public void setMode(DigitalChannelController.Mode mode) {
        }

        @Override
        public boolean getState() {
            assume("DigitalChannel.getState (beam breaks need the balls)");
            return false;
        }

        @Override
        public void setState(boolean state) {
        }
    }

    /** A servo. Position is what the code asked for; travel time is not modelled yet (UNMEASURED). */
    public static final class SimServo extends SimDevice implements Servo {
        private final int port;
        private Direction direction = Direction.FORWARD;
        private double position = 0.5;

        SimServo(String name, int port) {
            super(name);
            this.port = port;
        }

        @Override
        public ServoController getController() {
            assume("Servo.getController");
            return null;
        }

        @Override
        public int getPortNumber() {
            return port;
        }

        @Override
        public void setDirection(Direction newDirection) {
            direction = newDirection;
        }

        @Override
        public Direction getDirection() {
            return direction;
        }

        @Override
        public void setPosition(double newPosition) {
            position = newPosition;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public void scaleRange(double min, double max) {
            assume("Servo.scaleRange");
        }
    }

    /**
     * A motor that is not part of the drive: the shooter, the intake, the turret. Power makes it speed up toward power x top speed
     * with a lag, so getVelocity() (ticks per second) and getCurrentPosition() (ticks) answer from the virtual clock. Nothing here
     * moves a ball by itself: the world reads the intake power and the shooter speed when it works out pickups and shots.
     */
    public static final class SimAuxMotor extends SimDevice implements DcMotorEx {
        private final int port;
        private final double maxTicksPerSecond;
        private final double tauSeconds;
        private Direction direction = Direction.FORWARD;
        private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.BRAKE;
        private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
        private double power;
        private double velocity;   // ticks per second
        private double position;   // ticks
        private double offset;
        private long lastNanos = Long.MIN_VALUE;

        SimAuxMotor(String name, int port, double maxTicksPerSecond, double tauSeconds) {
            super(name);
            this.port = port;
            this.maxTicksPerSecond = maxTicksPerSecond;
            this.tauSeconds = tauSeconds;
        }

        /** Brings velocity and position up to the virtual time now, with the power that has been set since the last call. */
        private synchronized void update() {
            long now = SimClock.nanoTime();
            if (lastNanos == Long.MIN_VALUE) {
                lastNanos = now;
                return;
            }
            double dt = (now - lastNanos) / 1e9;
            lastNanos = now;
            if (dt <= 0) {
                return;
            }
            double target = Math.max(-1, Math.min(1, power)) * maxTicksPerSecond;
            double decay = Math.exp(-dt / tauSeconds);
            position += target * dt + (velocity - target) * tauSeconds * (1 - decay);
            velocity = target + (velocity - target) * decay;
        }

        @Override
        public synchronized void setPower(double newPower) {
            update();
            power = newPower;
        }

        @Override
        public synchronized double getPower() {
            return power;
        }

        @Override
        public synchronized double getVelocity() {
            update();
            return velocity;
        }

        @Override
        public double getVelocity(AngleUnit unit) {
            return getVelocity();
        }

        @Override
        public synchronized int getCurrentPosition() {
            update();
            return (int) Math.round(position - offset);
        }

        @Override
        public synchronized void setMode(RunMode newMode) {
            update();
            if (newMode == RunMode.STOP_AND_RESET_ENCODER) {
                offset = position;
            }
            if (newMode == RunMode.RUN_TO_POSITION) {
                assume(getDeviceName() + " RUN_TO_POSITION (the position target is not followed)");
            }
            mode = newMode;
        }

        @Override
        public RunMode getMode() {
            return mode;
        }

        @Override
        public void setDirection(Direction newDirection) {
            direction = newDirection; // the SDK flips power and encoder together, so the velocity the code reads is unchanged
        }

        @Override
        public Direction getDirection() {
            return direction;
        }

        @Override
        public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
            zeroPowerBehavior = behavior;
        }

        @Override
        public ZeroPowerBehavior getZeroPowerBehavior() {
            return zeroPowerBehavior;
        }

        @Override
        public int getPortNumber() {
            return port;
        }

        @Override
        public void setPowerFloat() {
            setPower(0);
        }

        @Override
        public boolean getPowerFloat() {
            return power == 0;
        }

        @Override
        public MotorConfigurationType getMotorType() {
            assume("DcMotor.getMotorType");
            return null;
        }

        @Override
        public void setMotorType(MotorConfigurationType motorType) {
        }

        @Override
        public DcMotorController getController() {
            assume("DcMotor.getController");
            return null;
        }

        @Override
        public void setTargetPosition(int position) {
            assume(getDeviceName() + ".setTargetPosition");
        }

        @Override
        public int getTargetPosition() {
            assume(getDeviceName() + ".getTargetPosition");
            return 0;
        }

        @Override
        public boolean isBusy() {
            assume(getDeviceName() + ".isBusy");
            return false;
        }

        @Override
        public void setMotorEnable() {
        }

        @Override
        public void setMotorDisable() {
        }

        @Override
        public boolean isMotorEnabled() {
            return true;
        }

        @Override
        public void setVelocity(double angularRate) {
            assume(getDeviceName() + ".setVelocity (the speed is not followed)");
        }

        @Override
        public void setVelocity(double angularRate, AngleUnit unit) {
            assume(getDeviceName() + ".setVelocity (the speed is not followed)");
        }

        @Override
        public void setPIDCoefficients(RunMode runMode, PIDCoefficients pidCoefficients) {
        }

        @Override
        public void setPIDFCoefficients(RunMode runMode, PIDFCoefficients pidfCoefficients) {
        }

        @Override
        public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        }

        @Override
        public void setPositionPIDFCoefficients(double p) {
        }

        @Override
        public PIDCoefficients getPIDCoefficients(RunMode runMode) {
            assume(getDeviceName() + ".getPIDCoefficients");
            return null;
        }

        @Override
        public PIDFCoefficients getPIDFCoefficients(RunMode runMode) {
            assume(getDeviceName() + ".getPIDFCoefficients");
            return null;
        }

        @Override
        public void setTargetPositionTolerance(int tolerance) {
        }

        @Override
        public int getTargetPositionTolerance() {
            return 0;
        }

        @Override
        public double getCurrent(CurrentUnit unit) {
            assume(getDeviceName() + ".getCurrent");
            return 0;
        }

        @Override
        public double getCurrentAlert(CurrentUnit unit) {
            return 0;
        }

        @Override
        public void setCurrentAlert(double current, CurrentUnit unit) {
        }

        @Override
        public boolean isOverCurrent() {
            return false;
        }
    }

    /** The beam breaks' analog controller: 3.3 V when the beam is clear, 0 V when a held ball blocks it. */
    private static final class SimBeamBreaks extends SimDevice implements AnalogInputController {
        private final SimWorld world;

        SimBeamBreaks(SimWorld world) {
            super("beam breaks");
            this.world = world;
        }

        @Override
        public double getAnalogInputVoltage(int channel) {
            return world.beamBlocked(channel) ? 0.0 : 3.3;
        }

        @Override
        public double getMaxAnalogInputVoltage() {
            return 3.3;
        }

        @Override
        public com.qualcomm.robotcore.util.SerialNumber getSerialNumber() {
            return null;
        }
    }
}
