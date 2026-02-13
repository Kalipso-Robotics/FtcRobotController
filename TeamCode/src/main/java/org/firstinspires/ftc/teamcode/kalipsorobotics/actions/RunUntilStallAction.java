package org.firstinspires.ftc.teamcode.kalipsorobotics.actions;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RunUntilStallAction extends Action {
    private final DcMotorEx motor;
    private final DcMotor regMotor;
    private final double power;
    private final double maxTimeoutMS;
    private int stallCount;
    private final ElapsedTime timeoutTimer;

    // goBILDA 5203 series motor specifications
    private static final double RPM = 1150.0; // RPM at 12V
    private static final double TICKS_PER_REV = ((1.0 + (46.0/11.0)) * 28.0); // ((1+(46/11)) * 28) = 145.09 ticks per revolution

    // Calculate max velocity: RPM * ticks/rev / 60 sec/min
    private static final double MAX_VELOCITY_TICKS_PER_SEC = RPM * TICKS_PER_REV / 60.0;

    // Stall detection thresholds
    private static final double STALL_CURRENT_THRESHOLD_MILLIAMPS = 5000.0;
    private static final double MIN_POWER_THRESHOLD = 0.5; // only checks for motors that run with a power over 0.5
    private static final double STALL_VELOCITY_PERCENTAGE = 0.3; // Consider stalled if velocity < 15% of expected

    public RunUntilStallAction(DcMotor motor, double power, double maxTimeoutMS) {
        this.motor = (DcMotorEx) motor;
        this.regMotor = motor;
        this.power = power;
        this.maxTimeoutMS = maxTimeoutMS;
        this.stallCount = 0;
        this.timeoutTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        if (isDone) {
            KLog.d("RunUntilStall", String.format("[%s] Already done, skipping update",
                getName() != null ? getName() : "unnamed"));
            return;
        }

        if (!hasStarted) {
            KLog.d("RunUntilStall", String.format("[%s] STARTING - Setting motor power to %.2f, timeout: %.1fs",
                getName() != null ? getName() : "unnamed", power, maxTimeoutMS / 1000.0));
            motor.setPower(power);
            hasStarted = true;
            timeoutTimer.reset();
        }

        motor.setPower(power);

        double elapsedTimeMs = timeoutTimer.milliseconds();
        double elapsedTimeSec = elapsedTimeMs / 1000.0;

        // Check timeout
        if (elapsedTimeMs > maxTimeoutMS) {
            KLog.d("RunUntilStall", String.format("[%s] TIMEOUT after %.1fs - Stopping motor",
                getName() != null ? getName() : "unnamed", elapsedTimeSec));
            regMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.2);
            isDone = true;
            return;
        }

        double curVelocity = Math.abs(motor.getVelocity());
        double currentDraw = motor.getCurrent(CurrentUnit.MILLIAMPS);
        double expectedVelocity = Math.abs(power) * MAX_VELOCITY_TICKS_PER_SEC;
        double stallVelocityThreshold = expectedVelocity * STALL_VELOCITY_PERCENTAGE;

        // Log motor state every 500ms
        if (((int)elapsedTimeMs) % 500 < 20) {
            KLog.d("RunUntilStall", String.format("[%s] Motor State - Time: %.1fs, Velocity: %.1f/%.1f (threshold: %.1f), Current: %.0fmA, StallCount: %d",
                getName() != null ? getName() : "unnamed",
                elapsedTimeSec,
                curVelocity,
                expectedVelocity,
                stallVelocityThreshold,
                currentDraw,
                stallCount));
        }

        // Check stall conditions
        boolean pastInitialDelay = elapsedTimeMs > 1000;
        boolean powerAboveThreshold = Math.abs(motor.getPower()) > MIN_POWER_THRESHOLD;
        boolean currentAboveThreshold = currentDraw > STALL_CURRENT_THRESHOLD_MILLIAMPS;
        boolean velocityBelowThreshold = curVelocity < stallVelocityThreshold;

        if (pastInitialDelay && powerAboveThreshold && currentAboveThreshold && velocityBelowThreshold) {
            stallCount++;
            if (stallCount == 1 || stallCount % 20 == 0) {
                KLog.d("RunUntilStall", String.format("[%s] STALL DETECTED (count: %d) - Current: %.0fmA > %.0fmA, Velocity: %.1f < %.1f",
                    getName() != null ? getName() : "unnamed",
                    stallCount,
                    currentDraw,
                    STALL_CURRENT_THRESHOLD_MILLIAMPS,
                    curVelocity,
                    stallVelocityThreshold));
            }
        } else {
            if (stallCount > 0) {
                KLog.d("RunUntilStall", String.format("[%s] Stall conditions cleared - Resetting count from %d (pastDelay:%b, power:%.2f>%.2f=%b, current:%.0f>%.0f=%b, vel:%.1f<%.1f=%b)",
                    getName() != null ? getName() : "unnamed",
                    stallCount,
                    pastInitialDelay,
                    Math.abs(motor.getPower()), MIN_POWER_THRESHOLD, powerAboveThreshold,
                    currentDraw, STALL_CURRENT_THRESHOLD_MILLIAMPS, currentAboveThreshold,
                    curVelocity, stallVelocityThreshold, velocityBelowThreshold));
            }
            stallCount = 0;
        }

        if (stallCount > 100) {
            KLog.d("RunUntilStall", String.format("[%s] STALL CONFIRMED (count: %d > 100) - Stopping motor after %.1fs",
                getName() != null ? getName() : "unnamed", stallCount, elapsedTimeSec));
            regMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.2);
            isDone = true;
        }
    }
}
