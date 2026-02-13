package org.firstinspires.ftc.teamcode.kalipsorobotics.test.localization;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.ArrayList;
import java.util.List;

@Disabled
//@Autonomous(name = "OdoAutoCalibrate_Robust", group = "Calibration")
public class OdoAutoCalibrate_Robust extends LinearOpMode {

    // ---- CONFIGURE THESE ----
    private DcMotorEx fLeft, fRight, bLeft, bRight;  // all drive motors
    private DcMotorEx leftOdo, rightOdo, backOdo;    // dead-wheel encoders (plugged to motor ports)
    private IMU imu;

    // Based on your Odometry.java constants: DEAD_WHEEL_RADIUS_MM = 24, TICKS_PER_REV = 2000
    private static final double TICKS_PER_MM = 2000.0 / (2.0 * Math.PI * 24.0); // ~13.26 ticks/mm

    private static final double BASE_SPIN_POWER = 0.22; // adjustable
    private static final int    RUNS = 4;               // CW/CCW pairs
    private static final double TARGET_DEG = 540.0;     // per run
    private static final double MAX_IMU_DELTA_RAD = Math.toRadians(6.0); // clamp IMU Δθ spikes per step

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Map hardware based on your DriveTrain configuration ---
        fLeft  = hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft  = hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");

        // Based on your DriveTrain.java: leftEncoder=bLeft, rightEncoder=bRight, backEncoder=fRight
        leftOdo  = bLeft;
        rightOdo = bRight;
        backOdo  = fRight;

        imu = hardwareMap.get(IMU.class, "imu");

        imu.resetYaw();

        telemetry.addLine("Robust Odo Calibration ready.");
        telemetry.addLine("Spins robot and uses regression to fit constants.");
        telemetry.addLine("Expected TRACK_WIDTH_MM ≈ 297");
        telemetry.addLine("Expected BACK_OFFSET_MM ≈ 70");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        double sumTW = 0, sumBO = 0;
        int successes = 0;

        // Alternate CW/CCW
        for (int k = 0; k < RUNS && opModeIsActive(); k++) {
            boolean cw = (k % 2 == 0); // even runs CW, odd runs CCW
            Result r = doOneSpinRun(cw);
            if (r.valid) {
                sumTW += r.trackWidthMM;
                sumBO += r.backOffsetMM;
                successes++;
            }
            telemetry.addData("Run", "%d / %d  valid=%s", (k+1), RUNS, r.valid);
            telemetry.addData("Run trackWidth(mm)", "%.2f", r.trackWidthMM);
            telemetry.addData("Run backOffset(mm)", "%.2f", r.backOffsetMM);
            telemetry.update();
            sleep(700);
        }

        telemetry.addLine("=== FINAL CALIBRATION (averaged) ===");
        if (successes > 0) {
            telemetry.addData("Successful runs", successes);
            telemetry.addData("AVG track_width_mm", "%.2f", sumTW / successes);
            telemetry.addData("AVG back_offset_mm", "%.2f", sumBO / successes);
            telemetry.addLine("Paste these into your Odometry.java:");
            telemetry.addLine(String.format("TRACK_WIDTH_MM = %.2f", sumTW / successes));
            telemetry.addLine(String.format("BACK_DISTANCE_TO_MID_ROBOT_MM = %.2f", -(sumBO / successes)));
        } else {
            telemetry.addLine("No valid runs. Check IMU, encoders, or increase TARGET_DEG.");
        }
        telemetry.update();
        sleep(5000);
    }

    // One CW or CCW run with regression fitting
    private Result doOneSpinRun(boolean cw) {
        // zero refs
        imu.resetYaw();
        sleep(150);

        int startL = leftOdo.getCurrentPosition();
        int startR = rightOdo.getCurrentPosition();
        int startB = backOdo.getCurrentPosition();

        double prevYawRad = 0.0;

        // data arrays
        List<Double> x_theta = new ArrayList<>();     // IMU Δθ
        List<Double> y_tw    = new ArrayList<>();     // (ΔL - ΔR) in mm
        List<Double> y_bo    = new ArrayList<>();     // ΔBack in mm

        // start spin with small power; sign picks direction
        double sign = cw ? 1.0 : -1.0;

        // Spin all wheels in tank drive mode
        fLeft.setPower( sign * BASE_SPIN_POWER);
        fRight.setPower(-sign * BASE_SPIN_POWER);
        bLeft.setPower( sign * BASE_SPIN_POWER);
        bRight.setPower(-sign * BASE_SPIN_POWER);

        double target = Math.toRadians(TARGET_DEG);

        // Closed-loop bias to suppress forward creep
        while (opModeIsActive()) {
            double yawDeg = getYawDeg();
            double yawRad = wrapRad(Math.toRadians(yawDeg));

            // stop condition
            if (Math.abs(yawRad) >= target) break;

            // read encoders
            int curL = leftOdo.getCurrentPosition();
            int curR = rightOdo.getCurrentPosition();
            int curB = backOdo.getCurrentPosition();

            double dL = (curL - startL) / TICKS_PER_MM;
            double dR = (curR - startR) / TICKS_PER_MM;
            double dB = (curB - startB) / TICKS_PER_MM;

            // IMU delta this step (unwrapped by subtraction + clamp spikes)
            double dTheta = wrapRad(yawRad - prevYawRad);
            if (Math.abs(dTheta) > MAX_IMU_DELTA_RAD) {
                // reject spike; skip sample
                prevYawRad = yawRad;
                correctiveCreepControl(dL, dR, sign);
                continue;
            }

            // capture sample
            x_theta.add(dTheta);
            y_tw.add(dL - dR);
            y_bo.add(dB);

            // small creep correction each loop (forward drift)
            correctiveCreepControl(dL, dR, sign);

            prevYawRad = yawRad;

            // slow loop a touch for stability
            sleep(10);
        }

        // stop
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

        // sanity: need sufficient rotation & samples
        double totalTheta = 0.0;
        for (double v : x_theta) totalTheta += Math.abs(v);

        if (totalTheta < Math.toRadians(120) || x_theta.size() < 50) {
            return new Result(false, 0, 0);
        }

        // Fit y = m*x + b via least squares
        // trackWidth slope m_tw from (ΔL-ΔR) vs Δθ
        Fit fitTW = fitSlopeIntercept(x_theta, y_tw);
        // backOffset slope m_bo from ΔBack vs Δθ  (backOffset = -m_bo)
        Fit fitBO = fitSlopeIntercept(x_theta, y_bo);

        double trackWidth = fitTW.m;     // units: mm  (since y_tw is mm, x is rad)
        double backOffset = -fitBO.m;    // note the minus sign

        return new Result(true, trackWidth, backOffset);
    }

    // Nudges power to reduce forward creep while spinning
    private void correctiveCreepControl(double dL_mm, double dR_mm, double spinSign) {
        double forward = 0.5 * (dL_mm + dR_mm);
        double k = 0.0008; // small gain; tune on your bot

        // if forward > 0 → creeping forward: reduce the side that's "pushing" forward
        double bias = k * forward;  // +/- small number

        double leftCmd  =  spinSign * BASE_SPIN_POWER - bias;
        double rightCmd = -spinSign * BASE_SPIN_POWER - bias;

        // clamp to [-0.5, 0.5] for safety
        leftCmd  = Math.max(-0.5, Math.min(0.5, leftCmd));
        rightCmd = Math.max(-0.5, Math.min(0.5, rightCmd));

        fLeft.setPower(leftCmd);
        fRight.setPower(rightCmd);
        bLeft.setPower(leftCmd);
        bRight.setPower(rightCmd);
    }

    // Simple LS fit
    private Fit fitSlopeIntercept(List<Double> x, List<Double> y) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int n = Math.min(x.size(), y.size());
        for (int i = 0; i < n; i++) {
            double xi = x.get(i), yi = y.get(i);
            sx += xi; sy += yi; sxx += xi*xi; sxy += xi*yi;
        }
        double denom = n*sxx - sx*sx;
        double m = (denom == 0) ? 0 : (n*sxy - sx*sy) / denom;
        double b = (n == 0) ? 0 : (sy - m*sx) / n;
        return new Fit(m, b);
    }

    private static class Fit {
        final double m, b;
        Fit(double m, double b) { this.m = m; this.b = b; }
    }

    private static class Result {
        final boolean valid;
        final double trackWidthMM, backOffsetMM;
        Result(boolean v, double tw, double bo) { valid = v; trackWidthMM = tw; backOffsetMM = bo; }
    }

    private double getYawDeg() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }

    private double wrapRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}