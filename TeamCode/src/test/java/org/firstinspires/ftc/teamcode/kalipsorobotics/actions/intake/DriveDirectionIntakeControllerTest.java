package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMechInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.function.BooleanSupplier;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.DriveDirectionIntakeController.pickSide;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide.FRONT;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide.BACK;

public class DriveDirectionIntakeControllerTest {

    private static final double FAST = 1e6;
    private static final double T = 100;

    private Servo pivotA, pivotB;
    private Servo flowerServo;
    private DcMotor motorA, motorB;
    private IntakeModule sideA, sideB;
    private FlowerMech flowerMech;
    private DriveDirectionIntakeController controller;
    private ForcedDetector detector;

    private double savedConfirmMs;

    @Before
    public void setUp() throws Exception {
        savedConfirmMs = IntakeConfig.SIDE_SWITCH_CONFIRM_MS;
        IntakeConfig.SIDE_SWITCH_CONFIRM_MS = 0; // instant switch unless a test sets it
        pivotA = FlowerMechInterlockTest.fakeServo();
        pivotB = FlowerMechInterlockTest.fakeServo();
        flowerServo = FlowerMechInterlockTest.fakeServo();
        motorA = FlowerMechInterlockTest.fakeMotor();
        motorB = FlowerMechInterlockTest.fakeMotor();
        sideA = new IntakeModule(FRONT, motorA, new KServo(pivotA, FAST, 255, 0, false), IntakeConfig.SIDE_A);
        sideB = new IntakeModule(BACK, motorB, new KServo(pivotB, FAST, 255, 0, false), IntakeConfig.SIDE_B);
        flowerMech = new FlowerMech(new KServo(flowerServo, 1e6, 255, 0, false), sideA);
        waitUntil(flowerMech::isUp);
        detector = new ForcedDetector();
        controller = new DriveDirectionIntakeController(new Intake(sideA, sideB, flowerMech), detector);
    }

    @After
    public void tearDown() {
        IntakeConfig.SIDE_SWITCH_CONFIRM_MS = savedConfirmMs;
        SharedData.resetOdometryWheelIMUVelocity();
        SharedData.setOdometryWheelIMUPosition(new Position(0, 0, 0));
    }

    @Test
    public void pickSide_table() {
        assertEquals(FRONT, pickSide(0, FRONT, T));      // default A at rest
        assertEquals(FRONT, pickSide(T + 1, BACK, T));  // forward flips to A
        assertEquals(BACK, pickSide(-T - 1, FRONT, T)); // backward flips to B
        assertEquals(BACK, pickSide(50, BACK, T));     // inside the band keeps last
        assertEquals(FRONT, pickSide(-50, FRONT, T));
        assertEquals(FRONT, pickSide(T, FRONT, T));      // exactly at threshold is inside the band
        assertEquals(BACK, pickSide(-T, BACK, T));
    }

    @Test
    public void driving_runsSelectedSide_andRaisesOther() throws Exception {
        drive(1000, 0, 0); // mm/s forward
        controller.update();
        waitUntil(sideA::isDown);
        assertEquals(FRONT, controller.getSelectedSide());
        assertEquals(IntakeConfig.intakePower, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(IntakeConfig.SIDE_B.pivotUpPos, pivotB.getPosition(), 1e-9);
    }

    @Test
    public void fieldVelocityIsRotatedByHeading() {
        // Facing +Y (theta 90 deg), moving toward -Y field = backward in the robot frame.
        drive(0, -1000, Math.PI / 2);
        controller.update();
        assertEquals(BACK, controller.getSelectedSide());
        assertEquals(IntakeConfig.intakePower, motorB.getPower(), 1e-9);
    }

    @Test
    public void full_stopsAndRaisesBothSides_evenWhenDrivingTheOtherWay() throws Exception {
        drive(1000, 0, 0);
        controller.update();
        waitUntil(sideA::isDown);
        detector.forceFull = true;
        controller.update();
        assertTrue(controller.isFull());
        assertEquals(0, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(IntakeConfig.SIDE_A.pivotUpPos, pivotA.getPosition(), 1e-9);
        assertEquals(IntakeConfig.SIDE_B.pivotUpPos, pivotB.getPosition(), 1e-9);

        drive(-1000, 0, 0); // driving backward must not lower B while full
        controller.update();
        assertEquals(FRONT, controller.getSelectedSide());
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(IntakeConfig.SIDE_B.pivotUpPos, pivotB.getPosition(), 1e-9);
    }

    @Test
    public void clearFull_intakesAgain() throws Exception {
        drive(1000, 0, 0);
        detector.forceFull = true;
        controller.update();
        assertEquals(0, motorA.getPower(), 1e-9);

        controller.clearFull(); // a shot, pass or reverse
        assertFalse(controller.isFull());
        controller.update();
        assertEquals(IntakeConfig.intakePower, motorA.getPower(), 1e-9);
    }

    /** Full on demand: the fake motors report no current. reset() (clearFull, or a side starting) clears it. */
    private static class ForcedDetector extends IntakeFullDetector {
        boolean forceFull = false;

        @Override
        public boolean isFull() {
            return forceFull;
        }

        @Override
        public void reset(long nowMs) {
            super.reset(nowMs);
            forceFull = false;
        }
    }

    @Test
    public void switchingToB_raisesAViaFlowerInterlock() throws Exception {
        drive(1000, 0, 0);
        controller.update();
        waitUntil(sideA::isDown);
        flowerMech.getServo().setTargetPosition(FlowerMech.DOWN_POS);
        waitUntil(flowerMech::isDown);

        drive(-1000, 0, 0);
        controller.update();
        assertEquals(BACK, controller.getSelectedSide());
        assertEquals(FlowerMech.UP_POS, flowerServo.getPosition(), 1e-9);
        assertEquals(IntakeConfig.SIDE_A.pivotDownPos, pivotA.getPosition(), 1e-9);
        waitUntil(() -> {
            controller.update();
            return pivotA.getPosition() == IntakeConfig.SIDE_A.pivotUpPos;
        });
    }

    @Test
    public void briefReverse_doesNotSwitch_sustainedReverseDoes() throws Exception {
        IntakeConfig.SIDE_SWITCH_CONFIRM_MS = 100;
        drive(1000, 0, 0);
        controller.update();
        assertEquals(FRONT, controller.getSelectedSide());

        drive(-1000, 0, 0); // bounce off the balls
        controller.update();
        assertEquals(FRONT, controller.getSelectedSide());
        drive(1000, 0, 0); // back to pushing forward: the timer restarts
        controller.update();
        Thread.sleep(150);
        drive(-1000, 0, 0);
        controller.update();
        assertEquals(FRONT, controller.getSelectedSide()); // old crossing must not count

        Thread.sleep(150); // held backward past the confirm time
        controller.update();
        assertEquals(BACK, controller.getSelectedSide());
    }

    private static void drive(double vxFieldMmPerS, double vyFieldMmPerS, double theta) {
        SharedData.setOdometryWheelIMUVelocity(new Velocity(vxFieldMmPerS / 1000, vyFieldMmPerS / 1000, 0));
        SharedData.setOdometryWheelIMUPosition(new Position(0, 0, theta));
    }

    private static void waitUntil(BooleanSupplier condition) throws InterruptedException {
        long deadline = System.currentTimeMillis() + 2000;
        while (!condition.getAsBoolean()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            Thread.sleep(1);
        }
    }
}
