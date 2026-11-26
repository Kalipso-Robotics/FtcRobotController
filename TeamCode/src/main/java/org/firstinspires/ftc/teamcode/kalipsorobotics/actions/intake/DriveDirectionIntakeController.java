package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Runs the intake side that faces the direction of travel (pattern: FieldOrientedDriveAction,
 * a live DriveController implementation). Full (the running side stalls, IntakeFullDetector): both sides stop and
 * raise and stay up, whatever the drive direction, until clearFull() (TeleOp: a shot, a pass or a reverse).
 */
public class DriveDirectionIntakeController implements IntakeController {
    public static final String TAG = "Intaking_IntakeController";

    private final Intake intake;
    private final IntakeFullDetector fullDetector;
    private IntakeSide selected = IntakeSide.A; // A is the default
    private boolean wasFull = false;
    private long switchSinceMs = -1; // when vx first crossed toward the other side; -1 = not crossing

    public DriveDirectionIntakeController(Intake intake) {
        this(intake, new IntakeFullDetector());
    }

    /** Tests pass a detector they control. */
    DriveDirectionIntakeController(Intake intake, IntakeFullDetector fullDetector) {
        this.intake = intake;
        this.fullDetector = fullDetector;
    }

    /** One threshold with keep-last is the deadband: inside +/-threshold the last side stays. */
    public static IntakeSide pickSide(double vxRobotMmPerS, IntakeSide last, double threshold) {
        if (vxRobotMmPerS > threshold) {
            return IntakeSide.A;
        }
        if (vxRobotMmPerS < -threshold) {
            return IntakeSide.B;
        }
        return last;
    }

    @Override
    public void update() {
        long now = System.currentTimeMillis();
        boolean full = fullDetector.isFull();
        if (full != wasFull) {
            wasFull = full;
            KLog.d(TAG, () -> "event=full value=" + full + " side=" + selected + " filteredMA=" + fullDetector.getFilteredMA());
        }
        if (full) {
            // Side selection is frozen while full, so driving the other way cannot lower a side again.
            switchSinceMs = -1;
            for (IntakeSide s : IntakeSide.values()) {
                intake.getModule(s).stop();
                intake.getModule(s).raise();
            }
        } else {
            // Odometry velocity is field frame in mm/ms: rotate by heading to robot forward, then to mm/s.
            Velocity v = SharedData.peekOdometryWheelIMUVelocity();
            double theta = SharedData.peekOdometryWheelIMUPosition().getTheta();
            double vx = (v.getX() * Math.cos(theta) + v.getY() * Math.sin(theta)) * 1000;
            IntakeSide next = pickSide(vx, selected, IntakeConfig.SIDE_SWITCH_VX_MM_PER_S);
            // Debounce: vx must stay past the threshold for SIDE_SWITCH_CONFIRM_MS, so a bump or bounce
            // (driving into balls / a wall) doesn't flip the side.
            if (next == selected) {
                switchSinceMs = -1;
            } else {
                if (switchSinceMs < 0) {
                    switchSinceMs = now;
                }
                if (now - switchSinceMs >= IntakeConfig.SIDE_SWITCH_CONFIRM_MS) {
                    IntakeSide from = selected;
                    selected = next;
                    switchSinceMs = -1;
                    KLog.d(TAG, () -> "event=switch from=" + from + " to=" + next + " vx=" + vx + " reason=driveDir");
                }
            }
            for (IntakeSide s : IntakeSide.values()) {
                if (s != selected) {
                    intake.getModule(s).stop();
                    intake.getModule(s).raise(); // the one line to change if the idle side should stay down
                }
            }
            IntakeModule running = intake.getModule(selected);
            if (running.getPower() != IntakeConfig.intakePower) {
                fullDetector.reset(now); // starting (side switch, or after a stop / reverse / shot): skip the spin-up spike
            }
            running.lower();
            running.run(IntakeConfig.intakePower);
            fullDetector.update(running.getCurrentMA(), running.getRPS(), now);
            KLog.d(TAG + "_debug", () -> "vx=" + Math.round(vx) + " side=" + selected + " filteredMA=" + Math.round(fullDetector.getFilteredMA())); // TEMP: side-flip tuning, remove after
        }
        intake.update(); // finishes flower-deferred raises
    }

    @Override
    public void clearFull() {
        if (fullDetector.isFull()) {
            KLog.d(TAG, "event=clearFull");
        }
        fullDetector.reset(System.currentTimeMillis());
    }

    @Override
    public boolean isFull() {
        return fullDetector.isFull();
    }

    @Override
    public IntakeSide getSelectedSide() {
        return selected;
    }
}
