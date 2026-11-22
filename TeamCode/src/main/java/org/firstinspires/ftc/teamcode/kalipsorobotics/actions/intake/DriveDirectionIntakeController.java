package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Runs the intake side that faces the direction of travel (pattern: FieldOrientedDriveAction,
 * a live DriveController implementation). Full: both sides stop and raise.
 */
public class DriveDirectionIntakeController implements IntakeController {
    public static final String TAG = "Intaking_IntakeController";

    private final Intake intake;
    private IntakeSide selected = IntakeSide.A; // A is the default
    private boolean wasFull = false;

    public DriveDirectionIntakeController(Intake intake) {
        this.intake = intake;
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
        boolean full = intake.isFull();
        if (full != wasFull) {
            wasFull = full;
            KLog.d(TAG, () -> "event=full value=" + full + " balls=" + intake.getBallCount());
        }
        if (full) {
            for (IntakeSide s : IntakeSide.values()) {
                intake.getSide(s).stop();
                intake.getSide(s).raise();
            }
        } else {
            // Odometry velocity is field frame in mm/ms: rotate by heading to robot forward, then to mm/s.
            Velocity v = SharedData.peekOdometryWheelIMUVelocity();
            double theta = SharedData.peekOdometryWheelIMUPosition().getTheta();
            double vx = (v.getX() * Math.cos(theta) + v.getY() * Math.sin(theta)) * 1000;
            IntakeSide next = pickSide(vx, selected, IntakeConfig.SIDE_SWITCH_VX_MM_PER_S);
            if (next != selected) {
                IntakeSide from = selected;
                selected = next;
                KLog.d(TAG, () -> "event=switch from=" + from + " to=" + next + " vx=" + vx + " reason=driveDir");
            }
            for (IntakeSide s : IntakeSide.values()) {
                if (s == selected) {
                    intake.getSide(s).lower();
                    intake.getSide(s).run(IntakeConfig.intakePower);
                } else {
                    intake.getSide(s).stop();
                    intake.getSide(s).raise(); // the one line to change if the idle side should stay down
                }
            }
        }
        intake.update(); // finishes flower-deferred raises
    }

    @Override
    public IntakeSide getSelectedSide() {
        return selected;
    }
}
