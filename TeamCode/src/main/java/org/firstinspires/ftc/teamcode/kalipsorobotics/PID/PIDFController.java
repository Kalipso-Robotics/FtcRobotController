package org.firstinspires.ftc.teamcode.kalipsorobotics.PID;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.function.DoubleSupplier;

/**
 * General purpose PIDF controller for any mechanism on the robot: flywheels, linear slides,
 * turrets, drivetrain axes and headings.
 *
 * <p>The output is always the sum of a feedback term and a feedforward term:
 *
 * <pre>
 *   output = kP*error + kI*&#8747;error + kD*d(error)/dt          (feedback)
 *          + kS*sign(motion) + kV*setpoint + kA*d(setpoint)/dt  (velocity feedforward)
 *          + gravity                                            (position feedforward)
 * </pre>
 *
 * <p>Which feedforward terms make sense depends on the mechanism, so the controller has a
 * {@link ControlMode}:
 * <ul>
 *   <li>{@link ControlMode#VELOCITY} - the setpoint <i>is</i> a speed (shooter RPS, drivetrain
 *       wheel velocity), so {@code kV * setpoint} is the base power that holds that speed.</li>
 *   <li>{@link ControlMode#POSITION} - the setpoint is a place to stop at (slide ticks, turret
 *       ticks, heading). {@code kV} is meaningless here; instead {@code kG} holds the mechanism
 *       against gravity and {@code kS} breaks static friction.</li>
 * </ul>
 *
 * <p>Everything else - output limits, integral anti-windup, derivative filtering, angle
 * wrapping, slew limiting, tolerance - is opt-in through {@link Builder} and defaults to
 * "safe and boring".
 *
 * <h2>Examples</h2>
 * <pre>
 * // Shooter flywheel: setpoint in RPS, motor only ever drives forward.
 * PIDFController flywheel = PIDFController.flywheel("shooter1", kP, kI, kD, kF).build();
 * motor.setPower(flywheel.calculate(currentRPS, targetRPS));
 *
 * // Linear slide: setpoint in encoder ticks, kG holds it up against gravity.
 * PIDFController slide = PIDFController.linearSlide("outtake", kP, kI, kD, kG)
 *         .tolerance(15, 20)
 *         .build();
 * slide.setpointReached();          // -> atSetpoint()
 *
 * // Turret: wraps the shortest way around, static friction breaks stiction near the target.
 * PIDFController turret = PIDFController.turret("turret", kP, kI, kD, kS)
 *         .continuousInput(-TICKS_PER_ROTATION / 2, TICKS_PER_ROTATION / 2)
 *         .build();
 *
 * // Drivetrain heading in radians.
 * PIDFController heading = PIDFController.heading("heading", kP, kI, kD).build();
 * </pre>
 */
public class PIDFController {

    /** How the setpoint should be interpreted, which decides the feedforward model. */
    public enum ControlMode {
        /** Setpoint is a position/angle to settle at. Uses kS and gravity feedforward. */
        POSITION,
        /** Setpoint is a speed to hold. Uses kS, kV and kA feedforward. */
        VELOCITY
    }

    /** How gravity pulls on the mechanism, if at all. */
    public enum GravityMode {
        /** Horizontal or gravity-neutral mechanism (turret, drivetrain). */
        NONE,
        /** Constant hold power, e.g. a vertical linear slide. */
        CONSTANT,
        /** {@code kG * cos(angle)}, e.g. a pivoting arm. Needs an angle supplier. */
        ARM_COSINE
    }

    // Loops faster than this give a meaningless derivative; slower than this means the loop
    // stalled (init, a long action, a hardware read) and the accumulated state is stale.
    private static final double MIN_LOOP_SECONDS = 1e-4;
    private static final double MAX_LOOP_SECONDS = 0.25;

    private static final String LOG_TAG = "PIDF";

    // ---------------------------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------------------------

    private final String name;
    private final ControlMode controlMode;

    private double kP;
    private double kI;
    private double kD;

    private double kS;  // static friction: constant push in the direction we want to move
    private double kV;  // velocity: power per unit of setpoint speed (the classic "kF")
    private double kA;  // acceleration: power per unit of setpoint change per second
    private double kG;  // gravity: power needed just to hold position

    private final GravityMode gravityMode;
    private final DoubleSupplier gravityAngleRad;

    private double minOutput;
    private double maxOutput;

    /** Hard cap on how much of the output the integral term may own. */
    private double maxIntegralOutput;
    /** Extra cap as a fraction of the feedforward, for velocity loops where FF does the work. */
    private final double maxIntegralFractionOfFeedforward;
    /** Only integrate when |error| is under this, so big moves do not wind up. */
    private final double integralZone;
    /** Same idea, relative to |setpoint|. Useful for velocity loops that span a wide range. */
    private final double integralZoneFraction;

    private final boolean derivativeOnMeasurement;
    /** 0 = raw derivative, closer to 1 = heavier low pass smoothing. */
    private final double derivativeSmoothing;

    private final boolean continuousInput;
    private final double inputRange;

    /** Below this error the feedback terms and kS are switched off, so the mechanism stops buzzing. */
    private final double errorDeadband;
    /** Max change in output per second, to stop current spikes on heavy mechanisms. */
    private final double maxOutputSlewPerSecond;

    private final double positionTolerance;
    private final double velocityTolerance;

    private final boolean logging;

    // ---------------------------------------------------------------------------------------
    // Runtime state
    // ---------------------------------------------------------------------------------------

    private boolean firstRun = true;
    private double lastTimeNanos;

    private double setpoint;
    private double measurement;
    private double error;
    private double errorRate;

    private double lastError;
    private double lastMeasurement;
    private double lastSetpoint;

    private double integralAccumulator;
    private double smoothedDerivative;
    private double lastOutput;

    // Last computed contribution of each term, purely for telemetry.
    private double proportionalTerm;
    private double integralTerm;
    private double derivativeTerm;
    private double feedforwardTerm;

    // ---------------------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------------------

    private PIDFController(Builder builder) {
        this.name = builder.name;
        this.controlMode = builder.controlMode;

        this.kP = builder.kP;
        this.kI = builder.kI;
        this.kD = builder.kD;
        this.kS = builder.kS;
        this.kV = builder.kV;
        this.kA = builder.kA;
        this.kG = builder.kG;

        this.gravityMode = builder.gravityMode;
        this.gravityAngleRad = builder.gravityAngleRad;

        this.minOutput = builder.minOutput;
        this.maxOutput = builder.maxOutput;

        this.maxIntegralOutput = builder.maxIntegralOutput;
        this.maxIntegralFractionOfFeedforward = builder.maxIntegralFractionOfFeedforward;
        this.integralZone = builder.integralZone;
        this.integralZoneFraction = builder.integralZoneFraction;

        this.derivativeOnMeasurement = builder.derivativeOnMeasurement;
        this.derivativeSmoothing = builder.derivativeSmoothing;

        this.continuousInput = builder.continuousInput;
        this.inputRange = builder.inputRange;

        this.errorDeadband = builder.errorDeadband;
        this.maxOutputSlewPerSecond = builder.maxOutputSlewPerSecond;

        this.positionTolerance = builder.positionTolerance;
        this.velocityTolerance = builder.velocityTolerance;

        this.logging = builder.logging;

        this.lastTimeNanos = SystemClock.elapsedRealtimeNanos();
    }

    /** Start configuring a controller. See the {@code flywheel}/{@code linearSlide}/... presets. */
    public static Builder builder(String name, ControlMode controlMode) {
        return new Builder(name, controlMode);
    }

    /**
     * Flywheel / shooter preset: velocity loop, forward-only power, feedforward carries the
     * output and the integral is only allowed to trim near the setpoint.
     */
    public static Builder flywheel(String name, double kP, double kI, double kD, double kV) {
        return builder(name, ControlMode.VELOCITY)
                .pid(kP, kI, kD)
                .velocityFeedforward(kV)
                .outputRange(0, 1)
                .integralZoneFraction(0.1)
                .maxIntegralFractionOfFeedforward(0.1)
                .derivativeSmoothing(0.6);
    }

    /**
     * Linear slide preset: position loop with a constant gravity hold, derivative taken on the
     * measurement so a new target does not kick the slide, and a slew limit so it does not
     * slam the belt.
     */
    public static Builder linearSlide(String name, double kP, double kI, double kD, double kG) {
        return builder(name, ControlMode.POSITION)
                .pid(kP, kI, kD)
                .gravity(kG)
                .outputRange(-1, 1)
                .derivativeOnMeasurement(true)
                .derivativeSmoothing(0.7)
                .maxIntegralOutput(0.25)
                .maxOutputSlewPerSecond(4.0);
    }

    /** Turret preset: position loop whose only feedforward is breaking static friction. */
    public static Builder turret(String name, double kP, double kI, double kD, double kS) {
        return builder(name, ControlMode.POSITION)
                .pid(kP, kI, kD)
                .staticFriction(kS)
                .outputRange(-1, 1)
                .derivativeOnMeasurement(true)
                .derivativeSmoothing(0.5)
                .maxIntegralOutput(0.2);
    }

    /** Drivetrain translation axis preset: position loop in inches/mm, no gravity. */
    public static Builder driveAxis(String name, double kP, double kI, double kD) {
        return builder(name, ControlMode.POSITION)
                .pid(kP, kI, kD)
                .outputRange(-1, 1)
                .derivativeOnMeasurement(true)
                .derivativeSmoothing(0.5)
                .maxIntegralOutput(0.2);
    }

    /** Drivetrain heading preset: position loop in radians that always turns the short way. */
    public static Builder heading(String name, double kP, double kI, double kD) {
        return driveAxis(name, kP, kI, kD)
                .continuousInput(-Math.PI, Math.PI);
    }

    /**
     * Legacy constructor kept so existing call sites keep compiling.
     * The control mode is inferred: a non-zero {@code F} means the setpoint is a speed.
     * Prefer {@link #builder(String, ControlMode)} or one of the presets for new code.
     */
    public PIDFController(double P, double I, double D, double F, double S, String controllerName) {
        this(legacyBuilder(P, I, D, F, S, controllerName));
    }

    /** @see #PIDFController(double, double, double, double, double, String) */
    public PIDFController(double P, double I, double D, double F, String controllerName) {
        this(P, I, D, F, 0, controllerName);
    }

    private static Builder legacyBuilder(double P, double I, double D, double F, double S, String name) {
        boolean velocityLoop = F != 0;
        Builder builder = velocityLoop
                ? flywheel(name, P, I, D, F)
                : builder(name, ControlMode.POSITION).pid(P, I, D).outputRange(-1, 1);
        return builder.staticFriction(S);
    }

    // ---------------------------------------------------------------------------------------
    // Control
    // ---------------------------------------------------------------------------------------

    /**
     * Run one iteration of the loop.
     *
     * @param measurement where the mechanism is now (ticks, RPS, radians, ...)
     * @param setpoint    where it should be, in the same units
     * @return output clamped to the configured output range, ready for {@code motor.setPower}
     */
    public double calculate(double measurement, double setpoint) {
        long now = SystemClock.elapsedRealtimeNanos();
        double dt = (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;
        return calculate(measurement, setpoint, dt);
    }

    /**
     * Run one iteration with an explicit timestep, for replaying logs or unit tests.
     *
     * @param dtSeconds seconds since the previous call
     */
    public double calculate(double measurement, double setpoint, double dtSeconds) {
        this.measurement = measurement;
        this.setpoint = setpoint;
        this.error = wrapError(setpoint - measurement);

        boolean timingValid = !firstRun && dtSeconds >= MIN_LOOP_SECONDS && dtSeconds <= MAX_LOOP_SECONDS;

        this.errorRate = computeErrorRate(measurement, dtSeconds, timingValid);
        this.feedforwardTerm = computeFeedforward(setpoint, dtSeconds, timingValid);

        boolean inDeadband = Math.abs(error) <= errorDeadband;

        double integralBeforeUpdate = integralAccumulator;
        if (!inDeadband) {
            updateIntegral(dtSeconds, timingValid);
        }

        double output = combineTerms(inDeadband);

        // Anti-windup: if we are already saturated and the error would push us further into
        // saturation, the integral cannot help, so undo this iteration's accumulation.
        double clamped = clamp(output, minOutput, maxOutput);
        if (clamped != output && Math.signum(error) == Math.signum(output)) {
            integralAccumulator = integralBeforeUpdate;
            output = combineTerms(inDeadband);
            clamped = clamp(output, minOutput, maxOutput);
        }

        clamped = applySlewLimit(clamped, dtSeconds, timingValid);

        logIteration(output, clamped, dtSeconds);
        saveState(clamped);
        return clamped;
    }

    /**
     * Run one iteration when only the error is known (target - current), for mechanisms that
     * track a relative offset rather than an absolute encoder value.
     * The setpoint is taken as zero, so velocity feedforward contributes nothing; position
     * feedforward (kS, gravity) still applies.
     */
    public double calculateFromError(double error) {
        return calculate(-error, 0);
    }

    /** @deprecated use {@link #calculateFromError(double)} - same behaviour, clearer name. */
    @Deprecated
    public double calculate(double error) {
        return calculateFromError(error);
    }

    /**
     * Legacy overload that also sets the output range. The range sticks for later calls;
     * prefer configuring it once with {@link Builder#outputRange(double, double)}.
     */
    public double calculate(double measurement, double setpoint, double minOutput, double maxOutput) {
        setOutputRange(minOutput, maxOutput);
        return calculate(measurement, setpoint);
    }

    /** Clear accumulated state. Call before starting a new motion, not every loop. */
    public void reset() {
        firstRun = true;
        integralAccumulator = 0;
        smoothedDerivative = 0;
        lastError = 0;
        lastMeasurement = 0;
        lastSetpoint = 0;
        lastOutput = 0;
        error = 0;
        errorRate = 0;
        lastTimeNanos = SystemClock.elapsedRealtimeNanos();
    }

    /** True once the mechanism is inside tolerance and has stopped moving. */
    public boolean atSetpoint() {
        return Math.abs(error) <= positionTolerance && Math.abs(errorRate) <= velocityTolerance;
    }

    // ---------------------------------------------------------------------------------------
    // Control internals
    // ---------------------------------------------------------------------------------------

    /** Shortest-path error for mechanisms that wrap around, e.g. a heading or a turret. */
    private double wrapError(double rawError) {
        if (!continuousInput) {
            return rawError;
        }
        double wrapped = rawError % inputRange;
        if (wrapped > inputRange / 2) {
            wrapped -= inputRange;
        } else if (wrapped < -inputRange / 2) {
            wrapped += inputRange;
        }
        return wrapped;
    }

    /**
     * Derivative input, smoothed by a low pass filter because encoder deltas are noisy.
     * Taking it on the measurement instead of the error avoids the spike ("derivative kick")
     * that a sudden setpoint change would otherwise produce.
     */
    private double computeErrorRate(double measurement, double dtSeconds, boolean timingValid) {
        if (!timingValid) {
            return smoothedDerivative;
        }
        double raw = derivativeOnMeasurement
                ? -wrapError(measurement - lastMeasurement) / dtSeconds
                : (error - lastError) / dtSeconds;
        smoothedDerivative = derivativeSmoothing * smoothedDerivative + (1 - derivativeSmoothing) * raw;
        return smoothedDerivative;
    }

    private double computeFeedforward(double setpoint, double dtSeconds, boolean timingValid) {
        double feedforward = 0;

        if (controlMode == ControlMode.VELOCITY) {
            feedforward += kS * Math.signum(setpoint);
            feedforward += kV * setpoint;
            if (kA != 0 && timingValid) {
                feedforward += kA * (setpoint - lastSetpoint) / dtSeconds;
            }
        } else if (Math.abs(error) > errorDeadband) {
            feedforward += kS * Math.signum(error);
        }

        switch (gravityMode) {
            case CONSTANT:
                feedforward += kG;
                break;
            case ARM_COSINE:
                feedforward += kG * Math.cos(gravityAngleRad.getAsDouble());
                break;
            case NONE:
            default:
                break;
        }
        return feedforward;
    }

    private void updateIntegral(double dtSeconds, boolean timingValid) {
        if (kI == 0 || !timingValid || !withinIntegralZone()) {
            return;
        }
        integralAccumulator += error * dtSeconds;

        double limit = integralOutputLimit();
        double maxAccumulator = limit / Math.abs(kI);
        integralAccumulator = clamp(integralAccumulator, -maxAccumulator, maxAccumulator);
    }

    private boolean withinIntegralZone() {
        double magnitude = Math.abs(error);
        return magnitude <= integralZone
                && magnitude <= Math.abs(setpoint) * integralZoneFraction;
    }

    /**
     * How much output the integral may own. Velocity loops usually cap it relative to the
     * feedforward, but that cap collapses to zero when there is no feedforward, so the
     * absolute cap always applies too.
     */
    private double integralOutputLimit() {
        double limit = maxIntegralOutput;
        if (maxIntegralFractionOfFeedforward < Double.POSITIVE_INFINITY && feedforwardTerm != 0) {
            limit = Math.min(limit, Math.abs(feedforwardTerm) * maxIntegralFractionOfFeedforward);
        }
        return limit;
    }

    private double combineTerms(boolean inDeadband) {
        if (inDeadband) {
            proportionalTerm = 0;
            integralTerm = 0;
            derivativeTerm = 0;
        } else {
            proportionalTerm = kP * error;
            integralTerm = clamp(kI * integralAccumulator, -integralOutputLimit(), integralOutputLimit());
            derivativeTerm = kD * errorRate;
        }
        return proportionalTerm + integralTerm + derivativeTerm + feedforwardTerm;
    }

    private double applySlewLimit(double output, double dtSeconds, boolean timingValid) {
        if (maxOutputSlewPerSecond == Double.POSITIVE_INFINITY || !timingValid) {
            return output;
        }
        double maxStep = maxOutputSlewPerSecond * dtSeconds;
        return clamp(output, lastOutput - maxStep, lastOutput + maxStep);
    }

    private void saveState(double output) {
        lastError = error;
        lastMeasurement = measurement;
        lastSetpoint = setpoint;
        lastOutput = output;
        firstRun = false;
    }

    private void logIteration(double rawOutput, double clampedOutput, double dtSeconds) {
        if (!logging || !KLog.isDebug()) {
            return;
        }
        final double loggedMeasurement = measurement;
        final double loggedSetpoint = setpoint;
        final double loggedError = error;
        final double loggedP = proportionalTerm;
        final double loggedI = integralTerm;
        final double loggedD = derivativeTerm;
        final double loggedF = feedforwardTerm;
        KLog.d(LOG_TAG, () -> String.format(
                "%s | current %.3f target %.3f error %.3f | P %.4f I %.4f D %.4f F %.4f "
                        + "| raw %.4f out %.4f | dt %.4f",
                name, loggedMeasurement, loggedSetpoint, loggedError,
                loggedP, loggedI, loggedD, loggedF, rawOutput, clampedOutput, dtSeconds));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ---------------------------------------------------------------------------------------
    // Live tuning
    // ---------------------------------------------------------------------------------------

    public double setKp(double value) {
        return kP = value;
    }

    public double setKi(double value) {
        return kI = value;
    }

    public double setKd(double value) {
        return kD = value;
    }

    /** Velocity feedforward, historically called kF. */
    public double setKf(double value) {
        return kV = value;
    }

    public void setKv(double value) {
        kV = value;
    }

    public void setKa(double value) {
        kA = value;
    }

    public void setKs(double value) {
        kS = value;
    }

    public void setKg(double value) {
        kG = value;
    }

    public double chKp(double delta) {
        return kP += delta;
    }

    public double chKi(double delta) {
        return kI += delta;
    }

    public double chKd(double delta) {
        return kD += delta;
    }

    public double chKf(double delta) {
        return kV += delta;
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public void setMaxIntegralOutput(double maxIntegralOutput) {
        this.maxIntegralOutput = Math.abs(maxIntegralOutput);
    }

    // ---------------------------------------------------------------------------------------
    // Readouts
    // ---------------------------------------------------------------------------------------

    public double getKp() {
        return kP;
    }

    public double getKi() {
        return kI;
    }

    public double getKd() {
        return kD;
    }

    /** @see #setKf(double) */
    public double getKf() {
        return kV;
    }

    public double getKv() {
        return kV;
    }

    public double getKa() {
        return kA;
    }

    public double getKs() {
        return kS;
    }

    public double getKg() {
        return kG;
    }

    public String getName() {
        return name;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getError() {
        return error;
    }

    /** Rate of change of the error, already filtered. Useful for "has it settled" checks. */
    public double getErrorRate() {
        return errorRate;
    }

    public double getLastOutput() {
        return lastOutput;
    }

    public double getProportionalTerm() {
        return proportionalTerm;
    }

    public double getIntegralTerm() {
        return integralTerm;
    }

    public double getDerivativeTerm() {
        return derivativeTerm;
    }

    public double getFeedforwardTerm() {
        return feedforwardTerm;
    }

    /** One line of live state for driver station telemetry. */
    public String debugString() {
        return String.format("%s err %.3f | P %.3f I %.3f D %.3f F %.3f | out %.3f",
                name, error, proportionalTerm, integralTerm, derivativeTerm, feedforwardTerm, lastOutput);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("%s [%s] kP %f, kI %f, kD %f, kV %f, kS %f, kA %f, kG %f",
                name, controlMode, kP, kI, kD, kV, kS, kA, kG);
    }

    // ---------------------------------------------------------------------------------------
    // Builder
    // ---------------------------------------------------------------------------------------

    /** Fluent configuration. Every option has a usable default, so set only what you need. */
    public static class Builder {
        private final String name;
        private final ControlMode controlMode;

        private double kP;
        private double kI;
        private double kD;
        private double kS;
        private double kV;
        private double kA;
        private double kG;

        private GravityMode gravityMode = GravityMode.NONE;
        private DoubleSupplier gravityAngleRad = () -> 0;

        private double minOutput = -1;
        private double maxOutput = 1;

        private double maxIntegralOutput = 1;
        private double maxIntegralFractionOfFeedforward = Double.POSITIVE_INFINITY;
        private double integralZone = Double.POSITIVE_INFINITY;
        private double integralZoneFraction = Double.POSITIVE_INFINITY;

        private boolean derivativeOnMeasurement = false;
        private double derivativeSmoothing = 0;

        private boolean continuousInput = false;
        private double inputRange = 0;

        private double errorDeadband = 0;
        private double maxOutputSlewPerSecond = Double.POSITIVE_INFINITY;

        private double positionTolerance = 0;
        private double velocityTolerance = Double.POSITIVE_INFINITY;

        private boolean logging = true;

        private Builder(String name, ControlMode controlMode) {
            this.name = name;
            this.controlMode = controlMode;
        }

        public Builder pid(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this;
        }

        /** Constant push needed to break static friction, applied in the direction of motion. */
        public Builder staticFriction(double kS) {
            this.kS = kS;
            return this;
        }

        /** Power per unit of setpoint speed. Velocity loops only. */
        public Builder velocityFeedforward(double kV) {
            this.kV = kV;
            return this;
        }

        /** Power per unit of setpoint acceleration. Velocity loops only. */
        public Builder accelerationFeedforward(double kA) {
            this.kA = kA;
            return this;
        }

        /** Constant power needed to hold the mechanism up, e.g. a vertical slide. */
        public Builder gravity(double kG) {
            return gravity(kG, GravityMode.CONSTANT, () -> 0);
        }

        /** {@code kG * cos(angle)} hold power for an arm, where 0 rad is horizontal. */
        public Builder armGravity(double kG, DoubleSupplier angleRad) {
            return gravity(kG, GravityMode.ARM_COSINE, angleRad);
        }

        public Builder gravity(double kG, GravityMode mode, DoubleSupplier angleRad) {
            this.kG = kG;
            this.gravityMode = mode;
            this.gravityAngleRad = angleRad;
            return this;
        }

        /** Output limits. Use {@code (0, 1)} for a mechanism that must never reverse. */
        public Builder outputRange(double minOutput, double maxOutput) {
            this.minOutput = minOutput;
            this.maxOutput = maxOutput;
            this.maxIntegralOutput = Math.max(Math.abs(minOutput), Math.abs(maxOutput));
            return this;
        }

        /** Hard cap on the integral's share of the output. Default: the output range. */
        public Builder maxIntegralOutput(double maxIntegralOutput) {
            this.maxIntegralOutput = Math.abs(maxIntegralOutput);
            return this;
        }

        /** Extra cap relative to the feedforward, for velocity loops. Default: none. */
        public Builder maxIntegralFractionOfFeedforward(double fraction) {
            this.maxIntegralFractionOfFeedforward = Math.abs(fraction);
            return this;
        }

        /** Only integrate once |error| is below this, so long moves do not wind up. */
        public Builder integralZone(double integralZone) {
            this.integralZone = Math.abs(integralZone);
            return this;
        }

        /** Same, expressed as a fraction of |setpoint|. Handy for wide-range velocity loops. */
        public Builder integralZoneFraction(double fraction) {
            this.integralZoneFraction = Math.abs(fraction);
            return this;
        }

        /** Differentiate the measurement instead of the error, removing setpoint-change kick. */
        public Builder derivativeOnMeasurement(boolean derivativeOnMeasurement) {
            this.derivativeOnMeasurement = derivativeOnMeasurement;
            return this;
        }

        /** Low pass on the derivative: 0 is raw, 0.9 is very smooth but laggy. */
        public Builder derivativeSmoothing(double derivativeSmoothing) {
            this.derivativeSmoothing = clamp(derivativeSmoothing, 0, 0.99);
            return this;
        }

        /**
         * Treat the input as circular so the controller always takes the short way round.
         * Use radians for a heading, or ticks-per-rotation for a continuous turret.
         */
        public Builder continuousInput(double minInput, double maxInput) {
            this.continuousInput = true;
            this.inputRange = maxInput - minInput;
            return this;
        }

        /** Errors smaller than this produce no feedback output, so the mechanism stops buzzing. */
        public Builder errorDeadband(double errorDeadband) {
            this.errorDeadband = Math.abs(errorDeadband);
            return this;
        }

        /** Limit how fast the output may change, in output units per second. */
        public Builder maxOutputSlewPerSecond(double maxOutputSlewPerSecond) {
            this.maxOutputSlewPerSecond = Math.abs(maxOutputSlewPerSecond);
            return this;
        }

        /** Error window that {@link #atSetpoint()} accepts. */
        public Builder tolerance(double positionTolerance) {
            return tolerance(positionTolerance, Double.POSITIVE_INFINITY);
        }

        /** Error and error-rate windows that {@link #atSetpoint()} accepts. */
        public Builder tolerance(double positionTolerance, double velocityTolerance) {
            this.positionTolerance = Math.abs(positionTolerance);
            this.velocityTolerance = Math.abs(velocityTolerance);
            return this;
        }

        /** Per-iteration KLog output. On by default; turn off for very hot loops. */
        public Builder logging(boolean logging) {
            this.logging = logging;
            return this;
        }

        public PIDFController build() {
            return new PIDFController(this);
        }
    }
}
