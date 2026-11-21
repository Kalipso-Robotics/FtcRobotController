package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.function.BooleanSupplier;

/**
 * The flower may be down only while its intake side is down. Servo travel is time-estimated by
 * KServo, so the pivots get a very fast speed (done within a few ms) and the flower a slower one
 * (tens of ms) so "the pivot has not moved yet" can be checked right after a raise.
 */
public class FlowerInterlockTest {

    private static final double FAST_DEG_PER_SEC = 1e6;
    private static final double FLOWER_DEG_PER_SEC = 1000;

    private Servo pivotServoA;
    private Servo flowerServo;
    private DcMotor motorA;
    private IntakeSide sideA;
    private Flower flower;

    @Before
    public void setUp() throws InterruptedException {
        pivotServoA = fakeServo();
        flowerServo = fakeServo();
        motorA = fakeMotor();
        sideA = new IntakeSide(BeamBreakPosition.IntakeSide.A, motorA, new KServo(pivotServoA, FAST_DEG_PER_SEC, 255, 0, false));
        flower = new Flower(new KServo(flowerServo, FLOWER_DEG_PER_SEC, 255, 0, false), sideA);
        waitUntil(flower::isUp);
    }

    @Test
    public void flowerRefusesToLower_whileSideIsUp() {
        assertFalse(flower.lower());
        assertFalse(flower.isDown());
        assertEquals(ModuleConfig.FLOWER_UP_POS, flowerServo.getPosition(), 1e-9);
    }

    @Test
    public void flowerLowers_onceSideIsDown() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);

        assertTrue(flower.lower());
        assertEquals(ModuleConfig.FLOWER_DOWN_POS, flowerServo.getPosition(), 1e-9);
        waitUntil(flower::isDown);
    }

    @Test
    public void raisingSideWithFlowerDown_raisesFlowerFirst_thenPivotOnLaterUpdate() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);
        assertTrue(flower.lower());
        waitUntil(flower::isDown);

        sideA.raise();
        assertEquals(ModuleConfig.FLOWER_UP_POS, flowerServo.getPosition(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_DOWN_POS, pivotServoA.getPosition(), 1e-9);
        assertFalse("flower must not go back down while its side is raising", flower.lower());

        // No further raise() from the caller: update() alone finishes the raise once the flower is up.
        waitUntil(() -> {
            sideA.update();
            return pivotServoA.getPosition() == ModuleConfig.INTAKE_PIVOT_A_UP_POS;
        });
        assertTrue(flower.isUp());
    }

    @Test
    public void sideWithoutFlower_raisesImmediately() throws InterruptedException {
        Servo pivotServoB = fakeServo();
        IntakeSide sideB = new IntakeSide(BeamBreakPosition.IntakeSide.B, fakeMotor(), new KServo(pivotServoB, FAST_DEG_PER_SEC, 255, 0, false));
        sideB.lower();
        waitUntil(sideB::isDown);

        sideB.raise();
        assertEquals(ModuleConfig.INTAKE_PIVOT_B_UP_POS, pivotServoB.getPosition(), 1e-9);
        assertFalse(sideB.isDown());
    }

    @Test
    public void runAndStop_setMotorPower() {
        sideA.run(0.7);
        assertEquals(0.7, motorA.getPower(), 1e-9);
        sideA.stop();
        assertEquals(0, motorA.getPower(), 1e-9);
    }

    @Test
    public void isUp_trueAtStart_falseWhenLowered_falseWhileRaisePending() throws InterruptedException {
        waitUntil(sideA::isUp);
        sideA.lower();
        assertFalse(sideA.isUp());
        waitUntil(sideA::isDown);
        assertTrue(flower.lower());
        waitUntil(flower::isDown);
        sideA.raise(); // deferred: flower is going up first
        assertFalse(sideA.isUp());
        waitUntil(() -> {
            sideA.update();
            return sideA.isUp();
        });
    }

    @Test
    public void setPivotPosition_refusesToLeaveDown_whileFlowerIsDown() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);
        assertTrue(flower.lower());
        waitUntil(flower::isDown);

        assertFalse(sideA.setPivotPosition(0.5));
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_DOWN_POS, pivotServoA.getPosition(), 1e-9);
    }

    @Test
    public void setPivotPosition_movesAndKeepsDownFlagInSync() throws InterruptedException {
        assertTrue(sideA.setPivotPosition(ModuleConfig.INTAKE_PIVOT_A_DOWN_POS));
        waitUntil(sideA::isDown);
        assertTrue(sideA.setPivotPosition(0.5));
        assertEquals(0.5, pivotServoA.getPosition(), 1e-9);
        assertFalse(sideA.isDown());
    }

    @Test
    public void flowerSetPosition_refusesNonUp_whileSideIsUp() {
        assertFalse(flower.setPosition(0.5));
        assertEquals(ModuleConfig.FLOWER_UP_POS, flowerServo.getPosition(), 1e-9);
        assertTrue(flower.setPosition(ModuleConfig.FLOWER_UP_POS));
    }

    @Test
    public void flowerSetPosition_moves_onceSideIsDown() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);
        assertTrue(flower.setPosition(0.5));
        assertEquals(0.5, flowerServo.getPosition(), 1e-9);
        assertFalse(flower.isUp());
    }

    @Test
    public void flowerSteppedBackToUp_countsAsUp_soPivotCanLeaveDown() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);
        double stepped = ModuleConfig.FLOWER_UP_POS + 0.05;
        assertTrue(flower.setPosition(stepped));
        for (int i = 0; i < 5; i++) stepped -= 0.01; // 0.19999999999999996: float steps miss UP exactly
        assertTrue(flower.setPosition(stepped));
        waitUntil(flower::isUp);
        assertTrue(sideA.setPivotPosition(0.5));
    }

    private static void waitUntil(BooleanSupplier condition) throws InterruptedException {
        long deadlineMs = System.currentTimeMillis() + 2000;
        while (!condition.getAsBoolean()) {
            assertTrue("timed out waiting for servo travel", System.currentTimeMillis() < deadlineMs);
            Thread.sleep(1);
        }
    }

    public static Servo fakeServo() {
        return fake(Servo.class, "setPosition", "getPosition");
    }

    public static DcMotor fakeMotor() {
        return fake(DcMotor.class, "setPower", "getPower");
    }

    /**
     * A device that remembers the last value given to its setter and returns it from its getter.
     * Every other method does nothing and returns zero/false/null.
     */
    private static <T> T fake(Class<T> type, String setter, String getter) {
        double[] value = {0};
        Object device = Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, (proxy, method, args) -> {
            if (method.getName().equals(setter)) {
                value[0] = (double) args[0];
                return null;
            }
            if (method.getName().equals(getter)) {
                return value[0];
            }
            Class<?> returnType = method.getReturnType();
            if (returnType == boolean.class) {
                return false;
            }
            if (returnType == int.class) {
                return 0;
            }
            if (returnType == double.class) {
                return 0.0;
            }
            return null;
        });
        return type.cast(device);
    }
}
