package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import static org.junit.Assert.assertNotNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

/** Spike: can the FTC SDK's HardwareMap and a LinearOpMode be created on the laptop? */
public class SdkSpikeTest {

    static class Dummy extends LinearOpMode {
        @Override
        public void runOpMode() {
        }
    }

    @Test
    public void hardwareMapCanBeCreated() {
        HardwareMap map = new HardwareMap(null, null);
        assertNotNull(map);
        assertNotNull(map.dcMotor);
    }

    @Test
    public void linearOpModeCanBeCreated() {
        assertNotNull(new Dummy());
    }
}
