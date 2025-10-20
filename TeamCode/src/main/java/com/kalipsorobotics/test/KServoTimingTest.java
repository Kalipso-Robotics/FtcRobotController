package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.Revolver;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "KServo Timing Test", group = "Test")
public class KServoTimingTest extends LinearOpMode {

    private enum TestState {
        WAITING,
        MOVING,
        COMPLETE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Revolver revolver = new Revolver(opModeUtilities);
        KServo servo = revolver.getRevolverServo();

        // Test positions
        double[] testPositions = {
                Revolver.REVOLVER_INDEX_0,  // 0.1
                Revolver.REVOLVER_INDEX_1,  // 0.5
                Revolver.REVOLVER_INDEX_2,  // 0.9
                Revolver.REVOLVER_INDEX_0   // back to 0.1
        };

        int currentTest = 0;
        TestState state = TestState.WAITING;
        long actualStartTime = 0;
        long actualTime = 0;
        double estimatedTime = 0;

        telemetry.addLine("KServo Timing Test");
        telemetry.addLine("Press A to start next move");
        telemetry.addLine("Press B to reset to index 0");
        telemetry.update();

        waitForStart();

        // Move to starting position
        servo.setTargetPosition(Revolver.REVOLVER_INDEX_0);
        sleep(2000);

        while (opModeIsActive()) {

            switch (state) {
                case WAITING:
                    telemetry.addLine("=== WAITING ===");
                    telemetry.addData("Current Position", "%.3f", servo.getPosition());
                    telemetry.addLine("");

                    if (currentTest < testPositions.length) {
                        telemetry.addData("Next Test", "%d of %d", currentTest + 1, testPositions.length);
                        telemetry.addData("From", "%.3f", servo.getPosition());
                        telemetry.addData("To", "%.3f", testPositions[currentTest]);
                        telemetry.addLine("");
                        telemetry.addLine("Press A to start move");
                    } else {
                        telemetry.addLine("All tests complete!");
                        telemetry.addLine("Press B to reset");
                    }

                    if (gamepad1.a && currentTest < testPositions.length) {
                        // Start the move
                        actualStartTime = System.currentTimeMillis();
                        servo.setTargetPosition(testPositions[currentTest]);
                        state = TestState.MOVING;

                        // Calculate expected time manually
                        double deltaPosition = Math.abs(testPositions[currentTest] - servo.getPosition());
                        estimatedTime = deltaPosition * 242 * (1000.0 / KServo.AXON_MAX_SPEED) * 1.25;

                        sleep(100); // debounce
                    }

                    if (gamepad1.b) {
                        currentTest = 0;
                        servo.setTargetPosition(Revolver.REVOLVER_INDEX_0);
                        sleep(1000);
                    }
                    break;

                case MOVING:
                    telemetry.addLine("=== MOVING ===");
                    telemetry.addData("Test", "%d of %d", currentTest + 1, testPositions.length);
                    telemetry.addData("Target", "%.3f", testPositions[currentTest]);
                    telemetry.addData("Current", "%.3f", servo.getPosition());
                    telemetry.addLine("");

                    long elapsed = System.currentTimeMillis() - actualStartTime;
                    telemetry.addData("Elapsed Time", "%d ms", elapsed);
                    telemetry.addData("Estimated Time", "%.0f ms", estimatedTime);
                    telemetry.addLine("");
                    telemetry.addData("isDone()", servo.isDone());

                    if (servo.isDone()) {
                        actualTime = System.currentTimeMillis() - actualStartTime;
                        state = TestState.COMPLETE;
                    }
                    break;

                case COMPLETE:
                    telemetry.addLine("=== COMPLETE ===");
                    telemetry.addData("Test", "%d of %d", currentTest + 1, testPositions.length);
                    telemetry.addData("Target", "%.3f", testPositions[currentTest]);
                    telemetry.addData("Final Position", "%.3f", servo.getPosition());
                    telemetry.addLine("");
                    telemetry.addData("Actual Time", "%d ms", actualTime);
                    telemetry.addData("Estimated Time", "%.0f ms", estimatedTime);

                    double error = actualTime - estimatedTime;
                    double errorPercent = (error / actualTime) * 100;
                    telemetry.addData("Error", "%d ms (%.1f%%)", (int)error, errorPercent);
                    telemetry.addLine("");

                    if (error < 0) {
                        telemetry.addLine("WARNING: Finished BEFORE estimate!");
                        telemetry.addLine("Servo may still be moving!");
                    } else if (error > 100) {
                        telemetry.addLine("NOTE: Finished much later than estimate");
                    } else {
                        telemetry.addLine("Good timing!");
                    }
                    telemetry.addLine("");
                    telemetry.addLine("Press A for next test");

                    if (gamepad1.a) {
                        currentTest++;
                        state = TestState.WAITING;
                        sleep(100); // debounce
                    }
                    break;
            }

            telemetry.update();
        }
    }
}