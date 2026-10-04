package org.firstinspires.ftc.teamcode.kalipsorobotics.localization;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.test.octoquad.OctoStartup;
import org.junit.Test;

/** Locks the OctoStartup gesture decisions against the 2026-09-22 bring-up record. */
public class OctoStartupLogicTest {

    @Test
    public void forwardPushKeepsWorkingDirections() {
        // ch0 +8321 with INVERT_X=false, ch2 +8247 with INVERT_X2=true: both already right.
        assertFalse(OctoStartup.suggestInvert(false, 8321));
        assertTrue(OctoStartup.suggestInvert(true, 8247));
    }

    @Test
    public void backwardsPodGetsFlipped() {
        assertTrue(OctoStartup.suggestInvert(false, -8321));
        assertFalse(OctoStartup.suggestInvert(true, -8247));
    }

    @Test
    public void leftTwistSetsMirror() {
        assertTrue(OctoStartup.suggestMirror(89));
        assertFalse(OctoStartup.suggestMirror(-89));
    }

    @Test
    public void leftPushCountsUpOnlyWhenMirrored() {
        // INVERT_Y=true, mirror=true, left push counted UP: correct, keep true.
        assertTrue(OctoStartup.suggestInvertY(true, 5000, true));
        // Counted DOWN under mirror: wrong, flip.
        assertFalse(OctoStartup.suggestInvertY(true, -5000, true));
        // Not mirrored: a left push must count DOWN.
        assertTrue(OctoStartup.suggestInvertY(true, -5000, false));
        assertFalse(OctoStartup.suggestInvertY(true, 5000, false));
        // Not mirrored, counted UP on a left push with INVERT_Y=false: wrong, flip to true.
        assertEquals(true, OctoStartup.suggestInvertY(false, 5000, false));
        assertEquals(false, OctoStartup.suggestInvertY(false, -5000, false));
    }
}
