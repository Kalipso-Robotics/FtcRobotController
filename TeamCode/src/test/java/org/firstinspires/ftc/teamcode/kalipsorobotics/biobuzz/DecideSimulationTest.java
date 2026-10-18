package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.PartnerMode;
import org.junit.Test;

import java.util.ArrayList;

/**
 * Plays the whole pretend match on the laptop, for every partner mode, using the real decide()
 * and the real PathPlanning. It prints every step, so run it and read the output.
 * (Android Studio: green arrow, then look at the Run window. Terminal: results are in
 * TeamCode/build/test-results/testDebugUnitTest/*DecideSimulationTest.xml.)
 */
public class DecideSimulationTest {

    /** Plays one whole match and returns every line it wrote. */
    private String playMatch(PartnerMode mode) {
        DecideSimulation simulation = new DecideSimulation();
        simulation.currentReport = new ArrayList<>();
        simulation.playWholeMatch(mode);

        StringBuilder text = new StringBuilder();
        for (String line : simulation.currentReport) {
            text.append(line).append('\n');
        }
        System.out.println(text);
        return text.toString();
    }

    @Test
    public void everyPartnerMode_playsToTheEnd_withAPathForEveryDrive() {
        for (PartnerMode mode : PartnerMode.values()) {
            String report = playMatch(mode);
            assertFalse(mode + ": a drive had no path", report.contains("NO PATH"));
            assertFalse(mode + ": ran out of time in the middle of a move",
                    report.contains("NOT ENOUGH TIME"));
            assertTrue(mode + ": the match never reached its end", report.contains("===== FINAL"));
        }
    }

    @Test
    public void everyPartnerMode_endsParked() {
        for (PartnerMode mode : PartnerMode.values()) {
            assertTrue(mode + ": did not park", playMatch(mode).contains("PARK. Match over"));
        }
    }

    @Test
    public void everyPartnerMode_tipsAtLeastOnce() {
        for (PartnerMode mode : PartnerMode.values()) {
            assertTrue(mode + ": never tipped", playMatch(mode).contains("HIVE TIPPED"));
        }
    }
}
