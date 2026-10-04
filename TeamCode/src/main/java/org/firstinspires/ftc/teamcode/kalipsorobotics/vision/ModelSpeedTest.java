package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

import android.content.Context;
import android.content.res.AssetFileDescriptor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Arrays;
import java.util.Locale;

/**
 * Times one TFLite model at a time on the Control Hub.
 * INIT, then PLAY. Dpad up/down picks a model, A runs it for 30 seconds.
 * Measures interpreter.run() only (no camera, no resize, no drawing),
 * same as TfliteBenchmark, so the numbers are directly comparable.
 */
@TeleOp(name = "Model Speed Test", group = "test")
public class ModelSpeedTest extends LinearOpMode {

    private static final String[] MODELS = {
            "model_w8a32.tflite",
            "rect_192x320.tflite",
            "rect_224x384.tflite",
            "rect_256x416.tflite"
    };
    private static final long RUN_MS = 30000;

    @Override
    public void runOpMode() {
        String[] results = new String[MODELS.length];
        int selected = 0;
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastA = false;

        while (opModeInInit() || opModeIsActive()) {
            if (gamepad1.dpad_down && !lastDown) {
                selected = (selected + 1) % MODELS.length;
            }
            if (gamepad1.dpad_up && !lastUp) {
                selected = (selected + MODELS.length - 1) % MODELS.length;
            }
            boolean startRun = opModeIsActive() && gamepad1.a && !lastA;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastA = gamepad1.a;

            if (startRun) {
                results[selected] = benchmark(MODELS[selected]);
            }

            telemetry.addLine("Dpad up/down = choose, press PLAY, then A = run 30 s");
            for (int i = 0; i < MODELS.length; i++) {
                String marker = (i == selected) ? "> " : "  ";
                String result = (results[i] == null) ? "not run yet" : results[i];
                telemetry.addLine(marker + MODELS[i] + "  ->  " + result);
            }
            telemetry.update();
            sleep(50);
        }
    }

    private String benchmark(String file) {
        Interpreter interpreter;
        try {
            Interpreter.Options options = new Interpreter.Options();
            options.setNumThreads(4);
            interpreter = new Interpreter(loadModel(hardwareMap.appContext, file), options);
        } catch (IOException e) {
            return "FAILED TO LOAD: " + e.getMessage();
        }

        ByteBuffer input = ByteBuffer.allocateDirect(interpreter.getInputTensor(0).numBytes());
        input.order(ByteOrder.nativeOrder());
        ByteBuffer output = ByteBuffer.allocateDirect(interpreter.getOutputTensor(0).numBytes());
        output.order(ByteOrder.nativeOrder());
        String shape = Arrays.toString(interpreter.getInputTensor(0).shape());

        // A few warm-up runs so first-run setup time doesn't skew the average
        for (int i = 0; i < 3; i++) {
            input.rewind();
            output.rewind();
            interpreter.run(input, output);
        }

        long start = System.currentTimeMillis();
        long totalNs = 0;
        int runs = 0;
        while (opModeIsActive() && System.currentTimeMillis() - start < RUN_MS) {
            input.rewind();
            output.rewind();
            long t0 = System.nanoTime();
            interpreter.run(input, output);
            totalNs += System.nanoTime() - t0;
            runs++;
            if (runs % 5 == 0) {
                telemetry.addData("Running", file + " " + shape);
                telemetry.addData("Runs", runs);
                telemetry.addData("Mean ms so far", "%.1f", totalNs / 1e6 / runs);
                telemetry.update();
            }
        }
        interpreter.close();

        if (runs == 0) {
            return "stopped before any runs";
        }
        double meanMs = totalNs / 1e6 / runs;
        return String.format(Locale.US, "%s %.1f ms (%.1f fps), %d runs",
                shape, meanMs, 1000.0 / meanMs, runs);
    }

    private static MappedByteBuffer loadModel(Context context, String file) throws IOException {
        try (AssetFileDescriptor fd = context.getAssets().openFd(file);
             FileInputStream in = new FileInputStream(fd.getFileDescriptor())) {
            return in.getChannel().map(FileChannel.MapMode.READ_ONLY,
                    fd.getStartOffset(), fd.getDeclaredLength());
        }
    }
}
