package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.beambreak;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.BeamBreakConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

/**
 * Ball counting over N sensors. Blocked is low voltage (BeamBreakConfig.BLOCKED_IS_LOW_VOLTAGE).
 * Each pattern is read at t=0 (filters seed) and again at t=1000, after CORRECTION_SETTLE_MS, so
 * the plausibility pass runs. Patterns are written A end first: X blocked, . clear.
 */
public class BeamBreakArrayTest {

    private static final double BLOCKED_V = 0.5;
    private static final double CLEAR_V = 3.0;

    private static BeamBreakArray read(String pattern, int capacity) {
        double[] volts = new double[pattern.length()];
        List<BeamBreakSensor> sensors = new ArrayList<>();
        AnalogInputController controller = (AnalogInputController) Proxy.newProxyInstance(
                AnalogInputController.class.getClassLoader(), new Class<?>[]{AnalogInputController.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("getAnalogInputVoltage")) {
                        return volts[(Integer) args[0]];
                    }
                    Class<?> type = method.getReturnType();
                    if (type == double.class) return 3.3;
                    if (type == int.class) return 0;
                    if (type == boolean.class) return false;
                    return null;
                });
        for (int i = 0; i < pattern.length(); i++) {
            volts[i] = pattern.charAt(i) == 'X' ? BLOCKED_V : CLEAR_V;
            IntakeSide side = i < pattern.length() / 2
                    ? IntakeSide.FRONT : IntakeSide.BACK;
            sensors.add(new BeamBreakSensor(new BeamBreakPosition("beam" + i, side, 1.6),
                    new AnalogInput(controller, i)));
        }
        BeamBreakArray array = new BeamBreakArray(sensors, capacity);
        array.update(0);
        array.update(1000);
        return array;
    }

    @Test
    public void defaultConfig_isSixSensorsForFourBalls() {
        assertEquals(6, BeamBreakConfig.SENSORS.size());
        assertEquals(4, BeamBreakConfig.CAPACITY);
    }

    @Test
    public void allClear_isEmpty() {
        BeamBreakArray array = read("......", 4);
        assertEquals(0, array.getCount());
        assertTrue(array.isEmpty());
        assertTrue(array.isPatternPlausible());
    }

    @Test
    public void ballsAtBothEnds_countPartial() {
        BeamBreakArray array = read("X....X", 4);
        assertEquals(2, array.getCount());
        assertFalse(array.isFull());
        assertTrue(array.isPatternPlausible());
    }

    @Test
    public void capacityReached_isFull_andPublishedToSharedData() {
        BeamBreakArray array = read("XX..XX", 4);
        assertEquals(4, array.getCount());
        assertTrue(array.isFull());
        assertEquals(4, SharedData.getBallCount());
        assertTrue(SharedData.getBeamBreakData().isFull());
    }

    @Test
    public void moreBlockedThanCapacity_clampsToCapacity() {
        BeamBreakArray array = read("XXX.XX", 4);
        assertEquals(4, array.getCount());
        assertEquals(5, array.getMeasuredCount());
    }

    @Test
    public void impossiblePattern_keepsLongestClearRun() {
        // Two clear runs can't happen; position 1 is a ball reading clear through its hole.
        BeamBreakArray array = read("X.X...", 4);
        assertFalse(array.isPatternPlausible());
        assertEquals(1, array.getCorrectionCount());
        assertTrue(array.isInferredBlocked(1));
        assertEquals(3, array.getCount());
        assertEquals("[X+X...]", array.getOccupancyString());
    }

    @Test
    public void differentSensorCount_countsTheSameWay() {
        assertEquals(4, read("XXXX", 4).getCount());
        assertTrue(read("XXXX", 4).isFull());
        assertEquals(3, read("X.X.", 4).getCount());
        assertEquals(2, read("X.......X", 3).getCount());
    }
}
