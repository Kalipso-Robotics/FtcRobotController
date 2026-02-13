package com.kalipsorobotics.test.shooter;

import com.kalipsorobotics.utilities.KFileWriter;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Unit Test for RampUpTimeInterpolation
 *
 * Tests the interpolation logic with:
 * - Exact matches (values in lookup table)
 * - Interpolated values (between table entries)
 * - Edge cases (below min, above max)
 * - Realistic RPS values commonly used in shooting
 *
 * Outputs results to CSV with expected vs actual values
 */
@TeleOp(name = "RampUpTime Interpolation Test", group = "Test")
@Disabled
public class RampUpTimeInterpolationTest extends LinearOpMode {

    private KFileWriter fileWriter;
    private OpModeUtilities opModeUtilities;

    // Test tolerance for floating point comparison (1ms)
    private static final double TOLERANCE_MS = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== RampUpTimeInterpolation Unit Test ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize file writer
        opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        fileWriter = new KFileWriter("RampUpTimeInterpolationTest", opModeUtilities);

        // Write CSV header
        fileWriter.writeLine("testType,targetRPS,expectedTimeMs,actualTimeMs,difference,passed");

        telemetry.addLine("Initialization complete!");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start tests");
        telemetry.update();

        waitForStart();

        int totalTests = 0;
        int passedTests = 0;
        int failedTests = 0;

        // ========== TEST 1: EXACT MATCHES ==========
        telemetry.addLine("Running Test 1: Exact Matches...");
        telemetry.update();

        TestResult[] exactMatchTests = runExactMatchTests();
        for (TestResult result : exactMatchTests) {
            totalTests++;
            if (result.passed) passedTests++;
            else failedTests++;
        }

        // ========== TEST 2: INTERPOLATED VALUES ==========
        telemetry.addLine("Running Test 2: Interpolated Values...");
        telemetry.update();

        TestResult[] interpolationTests = runInterpolationTests();
        for (TestResult result : interpolationTests) {
            totalTests++;
            if (result.passed) passedTests++;
            else failedTests++;
        }

        // ========== TEST 3: EDGE CASES ==========
        telemetry.addLine("Running Test 3: Edge Cases...");
        telemetry.update();

        TestResult[] edgeCaseTests = runEdgeCaseTests();
        for (TestResult result : edgeCaseTests) {
            totalTests++;
            if (result.passed) passedTests++;
            else failedTests++;
        }

        // ========== TEST 4: REALISTIC SHOOTING VALUES ==========
        telemetry.addLine("Running Test 4: Realistic Shooting Values...");
        telemetry.update();

        TestResult[] realisticTests = runRealisticValueTests();
        for (TestResult result : realisticTests) {
            totalTests++;
            if (result.passed) passedTests++;
            else failedTests++;
        }

        // Close file
        fileWriter.close();

        // Display final results
        telemetry.clear();
        telemetry.addLine("=== TEST RESULTS ===");
        telemetry.addLine();
        telemetry.addData("Total Tests", totalTests);
        telemetry.addData("Passed", passedTests);
        telemetry.addData("Failed", failedTests);
        telemetry.addData("Success Rate", "%.1f%%", (passedTests * 100.0 / totalTests));
        telemetry.addLine();

        if (failedTests == 0) {
            telemetry.addLine("✓ ALL TESTS PASSED!");
        } else {
            telemetry.addLine("✗ SOME TESTS FAILED");
            telemetry.addLine("Check CSV for details");
        }

        telemetry.addLine();
        telemetry.addLine("File saved to:");
        telemetry.addLine("/sdcard/Android/data/");
        telemetry.addLine("com.qualcomm.ftcrobotcontroller/");
        telemetry.addLine("files/OdometryLog/");
        telemetry.update();

        // Keep telemetry visible
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Test 1: Exact Matches
     * Tests all values that exist exactly in the lookup table
     */
    private TestResult[] runExactMatchTests() {
        // Test exact RPS values from the lookup table
        double[][] exactTests = {
            // targetRPS, expectedTime
            {26.0, 2957},
            {27.0, 2955},
            {28.0, 2816},
            {29.0, 3157},
            {30.0, 3324},
            {31.0, 3156},
            {32.0, 3065},
            {33.0, 3147},
            {34.0, 3300},
            {35.0, 3152},
            {36.0, 3297},
            {37.0, 3235},
            {38.0, 3545},
            {39.0, 3157},
            {40.0, 3385},
            {41.0, 3705},
            {42.0, 3600},
            {43.0, 3608},
            {44.0, 3590},
            {45.0, 3612},
            {46.0, 3676},
            {47.0, 3608},
            {48.0, 3980},
            {49.0, 3544},
            {50.0, 3983},
        };

        TestResult[] results = new TestResult[exactTests.length];
        for (int i = 0; i < exactTests.length; i++) {
            double targetRPS = exactTests[i][0];
            double expectedTime = exactTests[i][1];
            results[i] = runSingleTest("ExactMatch", targetRPS, expectedTime);
        }

        return results;
    }

    /**
     * Test 2: Interpolation
     * Tests values between lookup table entries to verify linear interpolation
     */
    private TestResult[] runInterpolationTests() {
        // Calculate expected interpolated values manually
        // Formula: time = time1 * (delta2/deltaTotal) + time2 * (delta1/deltaTotal)

        double[][] interpolationTests = {
            // Between 26.0 (2957ms) and 26.5 (3015ms)
            // targetRPS=26.25: delta1=0.25, delta2=0.25, deltaTotal=0.5
            // expected = 2957*0.5 + 3015*0.5 = 2986
            {26.25, 2986.0},

            // Between 30.0 (3324ms) and 30.5 (2986ms)
            // targetRPS=30.2: delta1=0.2, delta2=0.3, deltaTotal=0.5
            // overRatio=0.3/0.5=0.6, underRatio=0.2/0.5=0.4
            // expected = 3324*0.6 + 2986*0.4 = 1994.4 + 1194.4 = 3188.8
            {30.2, 3188.8},

            // Between 35.0 (3152ms) and 35.5 (3070ms)
            // targetRPS=35.25: delta1=0.25, delta2=0.25, deltaTotal=0.5
            // expected = 3152*0.5 + 3070*0.5 = 3111
            {35.25, 3111.0},

            // Between 40.0 (3385ms) and 40.5 (3298ms)
            // targetRPS=40.3: delta1=0.3, delta2=0.2, deltaTotal=0.5
            // overRatio=0.2/0.5=0.4, underRatio=0.3/0.5=0.6
            // expected = 3385*0.4 + 3298*0.6 = 1354 + 1978.8 = 3332.8
            {40.3, 3332.8},

            // Between 45.0 (3612ms) and 45.5 (3441ms)
            // targetRPS=45.1: delta1=0.1, delta2=0.4, deltaTotal=0.5
            // overRatio=0.4/0.5=0.8, underRatio=0.1/0.5=0.2
            // expected = 3612*0.8 + 3441*0.2 = 2889.6 + 688.2 = 3577.8
            {45.1, 3577.8},

            // Between 28.5 (3083ms) and 29.0 (3157ms)
            // targetRPS=28.75: delta1=0.25, delta2=0.25, deltaTotal=0.5
            // expected = 3083*0.5 + 3157*0.5 = 3120
            {28.75, 3120.0},

            // Between 42.0 (3600ms) and 42.5 (3587ms)
            // targetRPS=42.2: delta1=0.2, delta2=0.3, deltaTotal=0.5
            // overRatio=0.3/0.5=0.6, underRatio=0.2/0.5=0.4
            // expected = 3600*0.6 + 3587*0.4 = 2160 + 1434.8 = 3594.8
            {42.2, 3594.8},
        };

        TestResult[] results = new TestResult[interpolationTests.length];
        for (int i = 0; i < interpolationTests.length; i++) {
            double targetRPS = interpolationTests[i][0];
            double expectedTime = interpolationTests[i][1];
            results[i] = runSingleTest("Interpolation", targetRPS, expectedTime);
        }

        return results;
    }

    /**
     * Test 3: Edge Cases
     * Tests boundary conditions and out-of-range values
     */
    private TestResult[] runEdgeCaseTests() {
        double[][] edgeCaseTests = {
            // Below minimum (should return minimum ramp time: 2957ms for 26.0 RPS)
            {20.0, 2957.0},
            {25.0, 2957.0},
            {25.9, 2957.0},

            // Above maximum (should return maximum ramp time: 3983ms for 50.0 RPS)
            {50.1, 3983.0},
            {55.0, 3983.0},
            {60.0, 3983.0},

            // Exact boundaries
            {26.0, 2957.0},  // Minimum
            {50.0, 3983.0},  // Maximum
        };

        TestResult[] results = new TestResult[edgeCaseTests.length];
        for (int i = 0; i < edgeCaseTests.length; i++) {
            double targetRPS = edgeCaseTests[i][0];
            double expectedTime = edgeCaseTests[i][1];
            results[i] = runSingleTest("EdgeCase", targetRPS, expectedTime);
        }

        return results;
    }

    /**
     * Test 4: Realistic Shooting Values
     * Tests common RPS values used in actual robot operation
     */
    private TestResult[] runRealisticValueTests() {
        double[][] realisticTests = {
            // Depot shooting (close range): typically 28-32 RPS
            {28.5, 3083.0},  // Exact match
            {29.5, 3104.0},  // Exact match
            {31.5, 3064.0},  // Exact match

            // Mid-range shooting: typically 35-40 RPS
            {35.5, 3070.0},  // Exact match
            {37.5, 3532.0},  // Exact match
            {38.5, 3203.0},  // Exact match
            {39.5, 3370.0},  // Exact match

            // Long-range shooting: typically 43-48 RPS
            {43.5, 3506.0},  // Exact match
            {44.5, 3754.0},  // Exact match
            {46.5, 3834.0},  // Exact match
            {47.5, 4132.0},  // Exact match

            // Custom interpolated values for fine-tuning
            // Between 36.0 (3297ms) and 36.5 (3299ms)
            {36.3, 3297.8}, // Almost identical values, interpolation should be smooth

            // Between 48.0 (3980ms) and 48.5 (3667ms)
            // targetRPS=48.25: delta1=0.25, delta2=0.25
            // expected = 3980*0.5 + 3667*0.5 = 3823.5
            {48.25, 3823.5},
        };

        TestResult[] results = new TestResult[realisticTests.length];
        for (int i = 0; i < realisticTests.length; i++) {
            double targetRPS = realisticTests[i][0];
            double expectedTime = realisticTests[i][1];
            results[i] = runSingleTest("Realistic", targetRPS, expectedTime);
        }

        return results;
    }

    /**
     * Run a single test case
     */
    private TestResult runSingleTest(String testType, double targetRPS, double expectedTime) {
        double actualTime = RampUpTimeInterpolation.lookupRampUpTime(targetRPS);
        double difference = Math.abs(actualTime - expectedTime);
        boolean passed = difference <= TOLERANCE_MS;

        // Log result to file
        String line = String.format("%s,%.2f,%.1f,%.1f,%.1f,%s",
            testType, targetRPS, expectedTime, actualTime, difference,
            passed ? "PASS" : "FAIL");
        fileWriter.writeLine(line);

        // Log to console
        if (!passed) {
            telemetry.addLine(String.format(
                "FAIL: %s RPS=%.2f | Expected=%.1f | Actual=%.1f | Diff=%.1f",
                testType, targetRPS, expectedTime, actualTime, difference));
            telemetry.update();
        }

        return new TestResult(testType, targetRPS, expectedTime, actualTime, difference, passed);
    }

    /**
     * Simple class to hold test results
     */
    private static class TestResult {
        String testType;
        double targetRPS;
        double expectedTime;
        double actualTime;
        double difference;
        boolean passed;

        TestResult(String testType, double targetRPS, double expectedTime,
                   double actualTime, double difference, boolean passed) {
            this.testType = testType;
            this.targetRPS = targetRPS;
            this.expectedTime = expectedTime;
            this.actualTime = actualTime;
            this.difference = difference;
            this.passed = passed;
        }
    }
}
