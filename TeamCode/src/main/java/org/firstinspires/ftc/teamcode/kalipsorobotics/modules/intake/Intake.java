
package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

    private final OpModeUtilities opModeUtilities;

    DcMotor intakeMotorFlower;
    DcMotor intakeMotorNonFlower;

    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.intakeMotorFlower = opModeUtilities.getHardwareMap().dcMotor.get("intake"); // todo config
        this.intakeMotorNonFlower = opModeUtilities.getHardwareMap().dcMotor.get("intake"); // todo config
        intakeMotorFlower.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorNonFlower.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getIntakeMotorFlower() {
        return intakeMotorFlower;
    }
    public DcMotor getIntakeMotorNonFlower() {
        return intakeMotorNonFlower;
    }

    public DcMotorEx getIntakeMotorFlowerEx() {
        return (DcMotorEx) intakeMotorFlower;
    }
    public DcMotorEx getIntakeMotorNonFlowerEx() {
        return (DcMotorEx) intakeMotorNonFlower;
    }

    /** Which intake faces the robot's front (+x). Flip this if the flower intake is on the back. */
    public static boolean FLOWER_IS_FRONT = true;

    /** Forward speed (mm/ms) below which the robot counts as stopped; 0.05 mm/ms = 50 mm/s. */
    public static double DIRECTION_DEADBAND_MM_PER_MS = 0.05;

    private DcMotor lastChosenIntake = null;

    /**
     * Picks the intake on the side the robot is driving toward: the field-frame velocity from
     * SharedData is rotated by the heading into the robot frame (x = forward). Only chooses; it does not power either motor.
     * Below the deadband (stopped or just strafing) it keeps the previous choice, starting with the front intake.
     */
    public DcMotor chooseIntake() {
        DcMotor front = FLOWER_IS_FRONT ? intakeMotorFlower : intakeMotorNonFlower;
        DcMotor back = FLOWER_IS_FRONT ? intakeMotorNonFlower : intakeMotorFlower;

        // SharedData velocity is field frame (both Odometry and OctoQuadOdo), so rotate it into the
        // robot frame by the heading: forward = vx*cos(theta) + vy*sin(theta).
        double heading = SharedData.peekOdometryWheelIMUPosition().getTheta();
        Velocity fieldVelocity = SharedData.peekOdometryWheelIMUVelocity();
        double forwardVelocity = fieldVelocity.getX() * Math.cos(heading) + fieldVelocity.getY() * Math.sin(heading);

        if (forwardVelocity > DIRECTION_DEADBAND_MM_PER_MS) {
            lastChosenIntake = front;
        } else if (forwardVelocity < -DIRECTION_DEADBAND_MM_PER_MS) {
            lastChosenIntake = back;
        } else if (lastChosenIntake == null) {
            lastChosenIntake = front;
        }
        return lastChosenIntake;
    }

}
