package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak.BeamBreakPosition;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KCRServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.function.BooleanSupplier;

/**
 * The flower may be down only while its intake side is down. Pivot travel is time-estimated by
 * KServo (very fast speed here); the CR flower is done FLOWER_MOVE_MS after its command (set to 30 ms here)
 * so "the pivot has not moved yet" can be checked right after a raise.
 */
public class FlowerMechInterlockTest {

    private static final double FAST_DEG_PER_SEC = 1e6;

    private Servo pivotServoA;
    private CRServo flowerServo;
    private final double flowerMs = IntakeConfig.FLOWER_MOVE_MS;
    private DcMotor motorA;
    private IntakeSide sideA;
    private FlowerMech flowerMech;

    @Before
    public void setUp() throws InterruptedException {
        pivotServoA = fakeServo();
        IntakeConfig.FLOWER_MOVE_MS = 30;
        flowerServo = fakeCRServo();
        motorA = fakeMotor();
        sideA = new IntakeSide(BeamBreakPosition.IntakeSide.A, motorA, new KServo(pivotServoA, FAST_DEG_PER_SEC, 255, 0, false));
        flowerMech = new FlowerMech(new KCRServo(flowerServo, false), sideA);
        waitUntil(flowerMech::isUp);
    }

    @After
    public void tearDown() {
        IntakeConfig.FLOWER_MOVE_MS = flowerMs;
    }

    @Test
    public void flowerRefusesToLower_whileSideIsUp() {
        assertFalse(flowerMech.lower());
        assertFalse(flowerMech.isDown());
        assertEquals(IntakeConfig.FLOWER_UP_POWER, flowerServo.getPower(), 1e-9);
    }

    @Test
    public void flowerLowers_onceSideIsDown() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);

        assertTrue(flowerMech.lower());
        assertEquals(IntakeConfig.FLOWER_DOWN_POWER, flowerServo.getPower(), 1e-9);
        waitUntil(flowerMech::isDown);
    }

    @Test
    public void raisingSideWithFlowerDown_raisesFlowerFirst_thenPivotOnLaterUpdate() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);
        assertTrue(flowerMech.lower());
        waitUntil(flowerMech::isDown);

        sideA.raise();
        assertEquals(IntakeConfig.FLOWER_UP_POWER, flowerServo.getPower(), 1e-9);
        assertEquals(ModuleConfig.INTAKE_PIVOT_A_DOWN_POS, pivotServoA.getPosition(), 1e-9);
        assertFalse("flower must not go back down while its side is raising", flowerMech.lower());

        // No further raise() from the caller: update() alone finishes the raise once the flower is up.
        waitUntil(() -> {
            sideA.update();
            return pivotServoA.getPosition() == ModuleConfig.INTAKE_PIVOT_A_UP_POS;
        });
        assertTrue(flowerMech.isUp());
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
        assertTrue(flowerMech.lower());
        waitUntil(flowerMech::isDown);
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
        assertTrue(flowerMech.lower());
        waitUntil(flowerMech::isDown);

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
    public void flowerSetPower_refusesTowardDown_whileSideIsUp() {
        assertFalse(flowerMech.setPower(IntakeConfig.FLOWER_DOWN_POWER));
        assertEquals(IntakeConfig.FLOWER_UP_POWER, flowerServo.getPower(), 1e-9);
        assertTrue(flowerMech.setPower(IntakeConfig.FLOWER_UP_POWER));
        assertTrue(flowerMech.setPower(0));
    }

    @Test
    public void flowerSetPower_towardDown_onceSideIsDown_thenBackUpLetsPivotLeave() throws InterruptedException {
        sideA.lower();
        waitUntil(sideA::isDown);
        assertTrue(flowerMech.setPower(IntakeConfig.FLOWER_DOWN_POWER));
        assertEquals(IntakeConfig.FLOWER_DOWN_POWER, flowerServo.getPower(), 1e-9);
        assertFalse(flowerMech.isUp());
        assertFalse(sideA.setPivotPosition(0.5));

        assertTrue(flowerMech.setPower(IntakeConfig.FLOWER_UP_POWER * 0.5));
        waitUntil(flowerMech::isUp);
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

    public static CRServo fakeCRServo() {
        return fake(CRServo.class, "setPower", "getPower");
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
