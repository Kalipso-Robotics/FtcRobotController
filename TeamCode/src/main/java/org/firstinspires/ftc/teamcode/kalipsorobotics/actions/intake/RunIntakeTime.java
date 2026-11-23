package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Runs the intake (both sides, or one side) at a power for a time, then stops it. */
public class RunIntakeTime extends Action {
    public static final String TAG = "Intaking_RunIntakeTime";

    private final Intake intake;
    private final IntakeSide side;
    private double timeMS;
    private final double power;
    private final ElapsedTime timer = new ElapsedTime();
    private final boolean lowerFirst;

    /** Both sides. */
    public RunIntakeTime(Intake intake, double timeMS, double power) {
        this(intake, null, timeMS, power);
    }

    /** One side only, e.g. feeding that side's balls to the shooter. Does not lower the side (PushBall relies on that). */
    public RunIntakeTime(IntakeSide side, double timeMS, double power) {
        this(null, side, timeMS, power, false);
    }

    /**
     * One side only; lowerFirst=true lowers that side when it starts (picking up).
     * Auto: {@code new RunIntakeTime(intake.getSide(BeamBreakPosition.IntakeSide.B), 2000, IntakeConfig.intakePower, true)}
     */
    public RunIntakeTime(IntakeSide side, double timeMS, double power, boolean lowerFirst) {
        this(null, side, timeMS, power, lowerFirst);
    }

    private RunIntakeTime(Intake intake, IntakeSide side, double timeMS, double power) {
        this(intake, side, timeMS, power, intake != null);
    }

    private RunIntakeTime(Intake intake, IntakeSide side, double timeMS, double power, boolean lowerFirst) {
        this.intake = intake;
        this.side = side;
        this.timeMS = timeMS;
        this.power = power;
        this.lowerFirst = lowerFirst;
    }

    private void setPower(double power) {
        if (side != null) {
            side.run(power);
        } else {
            intake.run(power);
        }
    }

    private String sideLabel() {
        return side != null ? side.getSide().name() : "both";
    }

    @Override
    protected void update() {
        if (!hasStarted) {
            timer.reset();
            hasStarted = true;
            if (lowerFirst) {
                if (intake != null) {
                    intake.lower(); // both = lower both
                } else {
                    side.lower();
                }
            }
            KLog.d(TAG, () -> "name=" + getName() + " side=" + sideLabel() + " event=start power=" + power + " timeMs=" + timeMS);
        }
        if (isDone) {
            return;
        }
        setPower(power);

        if (timer.milliseconds() > timeMS) {
            setPower(0);
            isDone = true;
            KLog.d(TAG, () -> "name=" + getName() + " side=" + sideLabel() + " event=done elapsedMs=" + timer.milliseconds());
        } else if (KLog.isDebug()) {
            KLog.d(TAG, () -> "name=" + getName() + " side=" + sideLabel() + " elapsedMs=" + timer.milliseconds());
        }
    }

    public void setTimeMS(double timeMS) {
        this.timeMS = timeMS;
    }
}
