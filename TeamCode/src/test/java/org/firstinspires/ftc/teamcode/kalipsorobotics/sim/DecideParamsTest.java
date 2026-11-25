package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.firstinspires.ftc.teamcode.kalipsorobotics.sim.RedTippingAuto.PartnerMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.sim.DecideParams;
import org.junit.Test;

public class DecideParamsTest {

    @Test
    public void overrideIsApplied() {
        DecideParams p = DecideParams.fromJson("{\"SPEED_MM_PER_SECOND\": 1500, \"SOTM\": true}", true);
        assertEquals(1500, p.SPEED_MM_PER_SECOND, 0);
        assertEquals(true, p.SOTM);
        assertEquals(2, p.SECONDS_TO_PICK_UP, 0); // untouched keys keep the default
    }

    @Test
    public void unknownKeyThrowsWhenStrict() {
        try {
            DecideParams.fromJson("{\"NOT_A_PARAM\": 1}", true);
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // ok
        }
    }

    @Test
    public void unknownKeyIsIgnoredWhenNotStrict() {
        assertEquals(1000, DecideParams.fromJson("{\"NOT_A_PARAM\": 1}", false).SPEED_MM_PER_SECOND, 0);
    }

    @Test
    public void enumParses() {
        assertEquals(PartnerMode.NOT_SOLO, DecideParams.fromJson("{\"PARTNER_MODE\": \"NOT_SOLO\"}", true).PARTNER_MODE);
    }

    @Test
    public void eachAutoLoadsItsResourceFile() {
        assertEquals(PartnerMode.SOLO_NOTHING, DecideParams.load("RedSoloAuto").PARTNER_MODE);
        assertEquals(PartnerMode.SOLO_PRELOAD, DecideParams.load("RedDuoLightAuto").PARTNER_MODE);
        assertEquals(PartnerMode.NOT_SOLO, DecideParams.load("RedDuoFullAuto").PARTNER_MODE);
        assertEquals(false, DecideParams.load("RedDuoFullAuto").SOTM);
    }
}
