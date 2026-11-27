package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.function.BooleanSupplier;

/**
 * The flower may be down only while its intake side is down: the side's raise() raises the flower (through its KServo)
 * first and holds the pivot until the flower's travel estimate is over. Pivot travel is very fast here; the flower
 * servo is slower (FLOWER_DEG_PER_SEC: ~125 ms for a full move), so "the pivot has not moved yet" can be checked
 * right after a raise. Lowering the flower itself is a plain KServoAutoAction / setTargetPosition (no interlock in
 * FlowerMech): callers only lower it while the side is down.
 */
public class FlowerMechInterlockTest {

    private static final double FAST_DEG_PER_SEC = 1e6;
    private static final double FLOWER_DEG_PER_SEC = 2550; // 255 deg range: 100 ms per full move, x1.25 margin in KServo

    private Servo pivotServoA;
    private Servo flowerServo;
    private DcMotor motorA;
    private IntakeModule sideA;
    private FlowerMech flowerMech;

    @Before
    public void setUp() throws InterruptedException {
        pivotServoA = fakeServo();
        flowerServo = fakeServo();
        motorA = fakeMotor();
        sideA = new IntakeModule(IntakeSide.FRONT, motorA, new KServo(pivotServoA, FAST_DEG_PER_SEC, 255, 0, false), IntakeConfig.SIDE_A);
        flowerMech = new FlowerMech(new KServo(flowerServo, FLOWER_DEG_PER_SEC, 255, 0, false), sideA);
        waitUntil(flowerMech::isUp);
    }

    private void lowerSideThenFlower() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);
        flowerMech.getServo().setTargetPosition(FlowerMech.DOWN_POS);
        waitUntil(() -> flowerMech.isDown() && flowerMech.getServo().isDone());
    }

    @Test
    public void raisingSideWithFlowerDown_raisesFlowerFirst_thenPivotOnLaterUpdate() throws InterruptedException {
        lowerSideThenFlower();

        sideA.raise();
        assertEquals(FlowerMech.UP_POS, flowerServo.getPosition(), 1e-9);
        assertEquals("pivot waits for the flower's travel", IntakeConfig.SIDE_A.pivotDownPos, pivotServoA.getPosition(), 1e-9);
        sideA.update();
        assertEquals("still travelling: pivot still down", IntakeConfig.SIDE_A.pivotDownPos, pivotServoA.getPosition(), 1e-9);

        // No further raise() from the caller: update() alone finishes the raise once the flower is up.
        waitUntil(() -> {
            sideA.update();
            return pivotServoA.getPosition() == IntakeConfig.SIDE_A.pivotUpPos;
        });
        assertTrue(flowerMech.isUp());
    }

    @Test
    public void sideWithoutFlower_raisesImmediately() throws InterruptedException {
        Servo pivotServoB = fakeServo();
        IntakeModule sideB = new IntakeModule(IntakeSide.BACK, fakeMotor(), new KServo(pivotServoB, FAST_DEG_PER_SEC, 255, 0, false), IntakeConfig.SIDE_B);
        sideB.lower();
        waitUntil(sideB::isDown);

        sideB.raise();
        assertEquals(IntakeConfig.SIDE_B.pivotUpPos, pivotServoB.getPosition(), 1e-9);
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
        lowerSideThenFlower();
        sideA.raise(); // deferred: flower is going up first
        assertFalse(sideA.isUp());
        waitUntil(() -> {
            sideA.update();
            return sideA.isUp();
        });
    }

    @Test
    public void setPivotPosition_refusesToLeaveDown_whileFlowerIsDown() throws InterruptedException {
        lowerSideThenFlower();

        assertFalse(sideA.setPivotPosition(0.5));
        assertEquals(IntakeConfig.SIDE_A.pivotDownPos, pivotServoA.getPosition(), 1e-9);
    }

    @Test
    public void setPivotPosition_movesAndKeepsDownFlagInSync() throws InterruptedException {
        assertTrue(sideA.setPivotPosition(IntakeConfig.SIDE_A.pivotDownPos));
        waitUntil(sideA::isDown);
        assertTrue(sideA.setPivotPosition(0.5));
        assertEquals(0.5, pivotServoA.getPosition(), 1e-9);
        assertFalse(sideA.isDown());
    }

    @Test
    public void setPivotPosition_waitsForFlowerTravel_afterFlowerCommandedUp() throws InterruptedException {
        lowerSideThenFlower();
        flowerMech.getServo().setTargetPosition(FlowerMech.UP_POS);
        assertFalse("flower still travelling up", sideA.setPivotPosition(0.5));
        waitUntil(() -> flowerMech.getServo().isDone());
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
