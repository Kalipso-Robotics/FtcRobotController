package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMechInterlockTest;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KCRServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.util.function.BooleanSupplier;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.DriveDirectionIntakeController.pickSide;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.A;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition.IntakeSide.B;

public class DriveDirectionIntakeControllerTest {

    private static final double FAST = 1e6;
    private static final double T = 100;

    private Servo pivotA, pivotB;
    private CRServo flowerServo;
    private DcMotor motorA, motorB;
    private IntakeSide sideA, sideB;
    private FlowerMech flowerMech;
    private DriveDirectionIntakeController controller;

    @Before
    public void setUp() throws Exception {
        SharedData.resetBeamBreakData();
        pivotA = FlowerMechInterlockTest.fakeServo();
        pivotB = FlowerMechInterlockTest.fakeServo();
        flowerServo = FlowerMechInterlockTest.fakeCRServo();
        motorA = FlowerMechInterlockTest.fakeMotor();
        motorB = FlowerMechInterlockTest.fakeMotor();
        sideA = new IntakeSide(A, motorA, new KServo(pivotA, FAST, 255, 0, false));
        sideB = new IntakeSide(B, motorB, new KServo(pivotB, FAST, 255, 0, false));
        flowerMech = new FlowerMech(new KCRServo(flowerServo, false), sideA);
        waitUntil(flowerMech::isUp);
        controller = new DriveDirectionIntakeController(new Intake(sideA, sideB, flowerMech));
    }

    @After
    public void tearDown() {
        SharedData.resetBeamBreakData();
        SharedData.resetOdometryWheelIMUVelocity();
        SharedData.setOdometryWheelIMUPosition(new Position(0, 0, 0));
    }

    @Test
    public void pickSide_table() {
        assertEquals(A, pickSide(0, A, T));      // default A at rest
        assertEquals(A, pickSide(T + 1, B, T));  // forward flips to A
        assertEquals(B, pickSide(-T - 1, A, T)); // backward flips to B
        assertEquals(B, pickSide(50, B, T));     // inside the band keeps last
        assertEquals(A, pickSide(-50, A, T));
        assertEquals(A, pickSide(T, A, T));      // exactly at threshold is inside the band
        assertEquals(B, pickSide(-T, B, T));
    }

    @Test
    public void driving_runsSelectedSide_andRaisesOther() throws Exception {
        drive(1000, 0, 0); // mm/s forward
        controller.update();
        waitUntil(sideA::isDown);
        assertEquals(A, controller.getSelectedSide());
        assertEquals(IntakeConfig.intakePower, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_UP_POS, pivotB.getPosition(), 1e-9);
    }

    @Test
    public void fieldVelocityIsRotatedByHeading() {
        // Facing +Y (theta 90 deg), moving toward -Y field = backward in the robot frame.
        drive(0, -1000, Math.PI / 2);
        controller.update();
        assertEquals(B, controller.getSelectedSide());
        assertEquals(IntakeConfig.intakePower, motorB.getPower(), 1e-9);
    }

    @Test
    public void full_stopsAndRaisesBothSides() throws Exception {
        drive(1000, 0, 0);
        controller.update();
        waitUntil(sideA::isDown);
        SharedData.setBeamBreakData(fullData());
        controller.update();
        assertEquals(0, motorA.getPower(), 1e-9);
        assertEquals(0, motorB.getPower(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_UP_POS, pivotA.getPosition(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_UP_POS, pivotB.getPosition(), 1e-9);
    }

    @Test
    public void switchingToB_raisesAViaFlowerInterlock() throws Exception {
        drive(1000, 0, 0);
        controller.update();
        waitUntil(sideA::isDown);
        assertTrue(flowerMech.lower());
        waitUntil(flowerMech::isDown);

        drive(-1000, 0, 0);
        controller.update();
        assertEquals(B, controller.getSelectedSide());
        assertEquals(IntakeConfig.FLOWER_UP_POWER, flowerServo.getPower(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_DOWN_POS, pivotA.getPosition(), 1e-9);
        waitUntil(() -> {
            controller.update();
            return pivotA.getPosition() == ModuleConfig.INTAKE_PIVOT_A_UP_POS;
        });
    }

    private static void drive(double vxFieldMmPerS, double vyFieldMmPerS, double theta) {
        SharedData.setOdometryWheelIMUVelocity(new Velocity(vxFieldMmPerS / 1000, vyFieldMmPerS / 1000, 0));
        SharedData.setOdometryWheelIMUPosition(new Position(0, 0, theta));
    }

    private static BeamBreakData fullData() throws Exception {
        Constructor<BeamBreakData> c = BeamBreakData.class.getDeclaredConstructor(int.class, int.class, int.class, int.class,
                int.class, int.class, int.class, boolean.class, boolean.class, boolean.class, long.class);
        c.setAccessible(true);
        return c.newInstance(4, 4, 4, 4, 0, 0, 0, true, false, true, 0L);
    }

    private static void waitUntil(BooleanSupplier condition) throws InterruptedException {
        long deadline = System.currentTimeMillis() + 2000;
        while (!condition.getAsBoolean()) {
            assertTrue("timed out", System.currentTimeMillis() < deadline);
            Thread.sleep(1);
        }
    }
}
