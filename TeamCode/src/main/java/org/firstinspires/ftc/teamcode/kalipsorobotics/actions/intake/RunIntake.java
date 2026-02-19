package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RunIntake extends Action {
    private final DcMotorEx motor;
    private final double fullPower;

    // goBILDA 5203 series motor specifications
    private static final double RPM = 1150.0;
    private static final double TICKS_PER_REV = ((1.0 + (46.0 / 11.0)) * 28.0);
    private static final double MAX_VELOCITY_TICKS_PER_SEC = RPM * TICKS_PER_REV / 60.0;

    // Stall detection thresholds
    private static final double STALL_CURRENT_THRESHOLD_MA = 3400;
    private static final double STALL_VELOCITY_PERCENTAGE = 0.5;
    private static final int STALL_CONFIRM_COUNT = 8;
    private static final double STALL_POWER = 1; //0.55
    private static final double RECOVERY_VELOCITY_PERCENTAGE = 0.3;

    // State
    private boolean isStalled = false;
    private int stallCount = 0;
    private int clearCount = 0;
    private final ElapsedTime runTimer = new ElapsedTime();

    public RunIntake(DcMotor motor, double power) {
        this.motor = (DcMotorEx) motor;
        this.fullPower = power;
    }

    public RunIntake(Intake intake) {
        this(intake.getIntakeMotorEx(), 1);
    }

    @Override
    public void update() {
        if (!hasStarted) {
            motor.setPower(fullPower);
            hasStarted = true;
            runTimer.reset();
            KLog.d("RunIntake", () -> String.format("[%s] STARTED at power %.2f",
                    getName() != null ? getName() : "unnamed", fullPower));
            return;
        }

        double velocity = Math.abs(motor.getVelocity());
        double currentMA = motor.getCurrent(CurrentUnit.MILLIAMPS);
        double expectedVelocity = Math.abs(fullPower) * MAX_VELOCITY_TICKS_PER_SEC;
        KLog.d("RunIntake", () -> String.format("Velocity %% %.2f", ((velocity / expectedVelocity) * 100)) + " current: " + currentMA + "mA");
        // Skip stall detection during first second (motor spin-up)
        if (runTimer.milliseconds() < 1000) {
            motor.setPower(fullPower);
            return;
        }

        if (!isStalled) {
            // RUNNING at full power — check for stall
            boolean highCurrent = currentMA > STALL_CURRENT_THRESHOLD_MA;
            boolean lowVelocity = velocity < expectedVelocity * STALL_VELOCITY_PERCENTAGE;

            if (highCurrent && lowVelocity) {
                stallCount++;
            } else {
                stallCount = 0;
            }

            if (stallCount >= STALL_CONFIRM_COUNT) {
                isStalled = true;
                stallCount = 0;
                clearCount = 0;
                motor.setPower(STALL_POWER);
                KLog.d("RunIntake", () -> String.format("[%s] STALL — dropping to %.2f power (current: %.0fmA, velocity: %.1f)",
                        getName() != null ? getName() : "unnamed", STALL_POWER, currentMA, velocity));
            } else {
                motor.setPower(fullPower);
            }
        } else {
            // STALLED at low power — check if obstruction cleared
            // At low power, expected velocity is lower
            double lowPowerExpectedVelocity = STALL_POWER * MAX_VELOCITY_TICKS_PER_SEC;
            boolean velocityRecovering = velocity > lowPowerExpectedVelocity * RECOVERY_VELOCITY_PERCENTAGE;
            boolean currentNormal = currentMA < STALL_CURRENT_THRESHOLD_MA;

            if (velocityRecovering && currentNormal) {
                clearCount++;
            } else {
                clearCount = 0;
            }

            if (clearCount >= STALL_CONFIRM_COUNT) {
                isStalled = false;
                clearCount = 0;
                motor.setPower(fullPower);
                KLog.d("RunIntake", () -> String.format("[%s] RECOVERED — back to full power %.2f (velocity: %.1f)",
                        getName() != null ? getName() : "unnamed", fullPower, velocity));
            } else {
                motor.setPower(STALL_POWER);
            }
        }
    }
}
