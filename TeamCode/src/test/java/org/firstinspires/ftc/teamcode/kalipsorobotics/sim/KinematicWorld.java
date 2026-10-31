package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.DrivetrainConfig;

/**
 * The robot's body. It takes the motor powers the code set and moves the robot, then reports what
 * the sensors would read: dead-wheel encoder ticks and the IMU heading.
 *
 * This is the plain drivetrain model: an open field, no walls, no balls, no Rapier. It is kept next to
 * RapierWorld as a cross-check. With nothing in the way the two must agree, so a gap between them
 * means a mistake in one of them.
 *
 * Frame: the same as Odometry. Field x, y in mm, theta in radians. Robot frame: x forward, y left.
 * The wheel mix is the one AdaptivePurePursuitAction uses, run backwards:
 *   FL = vx + vy + w,  BL = vx - vy + w,  FR = vx - vy - w,  BR = vx + vy - w
 * The pods are the ones Odometry decodes, run backwards:
 *   theta = (L - R) / trackWidth,  x = (L + R) / 2,  y = back - backDistance * theta
 *
 * UNMEASURED numbers are marked. They are guesses until fitted from a real odometry log.
 */
public final class KinematicWorld implements SimWorld {

    public static final int FL = 0, FR = 1, BL = 2, BR = 3;
    private static final String[] MOTOR_NAMES = {"fLeft", "fRight", "bLeft", "bRight"};

    // The same numbers as AdaptivePurePursuitAction.java (K_v at :140, wheelbase and track at :136-137).
    // The code's own feedforward says wheel speed (mm/s) = power / K_v.
    static final double K_V = 0.00036;
    private static final double WHEELBASE_MM = 232.414;
    private static final double TRACK_MM = 276.58;
    static final double K_OMEGA_MM_PER_RAD = Math.sqrt(WHEELBASE_MM * WHEELBASE_MM + TRACK_MM * TRACK_MM) / 2;

    // UNMEASURED: how fast a wheel reaches the speed a power asks for (seconds), coasting and braking.
    static final double WHEEL_RESPONSE_SECONDS = 0.08;
    static final double WHEEL_BRAKE_RESPONSE_SECONDS = 0.03;

    private static final long STEP_NANOS = 1_000_000L; // physics step: 1 ms

    // The TRUE robot. The defaults equal the numbers the code assumes, so the sim cannot see a wrong
    // constant in the code until these are measured on the real robot. UNMEASURED.
    static final double TRUE_TRACK_WIDTH_MM = DrivetrainConfig.TRACK_WIDTH_MM;
    static final double TRUE_BACK_DISTANCE_MM = DrivetrainConfig.BACK_DISTANCE_TO_MID_ROBOT_MM;
    static final double TRUE_MM_PER_TICK = DrivetrainConfig.MM_PER_TICK;

    private final double[] power = new double[4];
    private final boolean[] brakeWhenZero = {true, true, true, true};
    private final double[] wheelMmPerS = new double[4];
    private double fLeftWheelMm;

    private double x, y, theta;
    private double leftPodMm, rightPodMm, backPodMm;
    private long simNanos;

    @Override
    public synchronized void setPose(double xMm, double yMm, double thetaRad) {
        x = xMm;
        y = yMm;
        theta = thetaRad;
    }

    @Override
    public synchronized double getX() {
        return x;
    }

    @Override
    public synchronized double getY() {
        return y;
    }

    @Override
    public synchronized double getTheta() {
        return theta;
    }

    // ---- what the motors tell the world ----

    @Override
    public synchronized void setMotorPower(String motorName, double motorPower) {
        power[indexOf(motorName)] = motorPower;
    }

    @Override
    public synchronized void setBrake(String motorName, boolean brake) {
        brakeWhenZero[indexOf(motorName)] = brake;
    }

    // ---- what the sensors read ----

    /**
     * Raw encoder ticks of the encoder plugged into a motor port. The three dead-wheel pods are
     * plugged into bLeft (left pod), bRight (right pod) and fRight (back pod), as in DriveTrain.
     * Signs follow Odometry: left +, right -, back +.
     */
    @Override
    public synchronized double rawEncoderTicks(String motorName) {
        switch (indexOf(motorName)) {
            case BL:
                return leftPodMm / TRUE_MM_PER_TICK;
            case BR:
                return -rightPodMm / TRUE_MM_PER_TICK;
            case FR:
                return backPodMm / TRUE_MM_PER_TICK;
            default:
                return fLeftWheelMm / TRUE_MM_PER_TICK; // fLeft has no pod; this is its wheel
        }
    }

    /** IMU yaw in degrees, the way Odometry reads it: heading = -yaw. */
    @Override
    public synchronized double rawYawDegrees() {
        return -Math.toDegrees(theta);
    }

    // ---- physics ----

    @Override
    public synchronized void advanceTo(long toNanos) {
        while (simNanos < toNanos) {
            long stepNanos = Math.min(STEP_NANOS, toNanos - simNanos);
            step(stepNanos / 1e9);
            simNanos += stepNanos;
        }
    }

    private void step(double dt) {
        double maxWheelMmPerS = 1.0 / K_V;
        for (int i = 0; i < 4; i++) {
            double p = Math.max(-1, Math.min(1, power[i]));
            double responseSeconds = (p == 0 && brakeWhenZero[i]) ? WHEEL_BRAKE_RESPONSE_SECONDS : WHEEL_RESPONSE_SECONDS;
            double target = p * maxWheelMmPerS;
            wheelMmPerS[i] += (target - wheelMmPerS[i]) * (1 - Math.exp(-dt / responseSeconds));
        }
        fLeftWheelMm += wheelMmPerS[FL] * dt;

        double vx = (wheelMmPerS[FL] + wheelMmPerS[BL] + wheelMmPerS[FR] + wheelMmPerS[BR]) / 4;
        double vy = (wheelMmPerS[FL] - wheelMmPerS[BL] - wheelMmPerS[FR] + wheelMmPerS[BR]) / 4;
        double w = ((wheelMmPerS[FL] + wheelMmPerS[BL] - wheelMmPerS[FR] - wheelMmPerS[BR]) / 4) / K_OMEGA_MM_PER_RAD;

        double dx = vx * dt;
        double dy = vy * dt;
        double dTheta = w * dt;

        leftPodMm += dx + TRUE_TRACK_WIDTH_MM / 2 * dTheta;
        rightPodMm += dx - TRUE_TRACK_WIDTH_MM / 2 * dTheta;
        backPodMm += dy + TRUE_BACK_DISTANCE_MM * dTheta;

        double midTheta = theta + dTheta / 2;
        x += dx * Math.cos(midTheta) - dy * Math.sin(midTheta);
        y += dy * Math.cos(midTheta) + dx * Math.sin(midTheta);
        theta += dTheta;
    }

    private static int indexOf(String motorName) {
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            if (MOTOR_NAMES[i].equals(motorName)) {
                return i;
            }
        }
        throw new IllegalArgumentException("SimWorld does not know a drive motor named \"" + motorName + "\"");
    }
}
