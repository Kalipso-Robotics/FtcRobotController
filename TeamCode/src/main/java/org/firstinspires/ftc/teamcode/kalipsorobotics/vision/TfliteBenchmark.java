package org.firstinspires.ftc.teamcode.kalipsorobotics.vision;

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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Times raw TFLite interpreter throughput on the Control Hub.
 * No camera, no preprocessing, no decode - just interpreter.run() in a loop.
 * Runs each model in MODELS for RUN_SECONDS, then reports.
 */
@TeleOp(name = "TFLite Benchmark", group = "test")
public class TfliteBenchmark extends LinearOpMode {

    private static final String[] MODELS = {
            "best_float32.tflite",
            "yolo26n_int8.tflite"
    };

    private static final int RUN_SECONDS = 60;
    private static final int THREADS = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("press start to benchmark");
        telemetry.addData("models", Arrays.toString(MODELS));
        telemetry.addData("seconds each", RUN_SECONDS);
        telemetry.update();

        waitForStart();

        List<String> summary = new ArrayList<>();

        for (String name : MODELS) {
            if (!opModeIsActive()) break;
            summary.add(benchmark(name, summary));
        }

        while (opModeIsActive()) {
            telemetry.addLine("DONE");
            for (String line : summary) telemetry.addLine(line);
            telemetry.update();
            sleep(500);
        }
    }

    private String benchmark(String modelName, List<String> done) {
        Interpreter interp;
        int inBytes, outBytes;
        String shapeInfo;

        try {
            MappedByteBuffer model = loadAsset(modelName);
            Interpreter.Options opts = new Interpreter.Options();
            opts.setNumThreads(THREADS);
            interp = new Interpreter(model, opts);

            inBytes = interp.getInputTensor(0).numBytes();
            outBytes = interp.getOutputTensor(0).numBytes();
            shapeInfo = Arrays.toString(interp.getInputTensor(0).shape())
                    + " " + interp.getInputTensor(0).dataType()
                    + " -> " + Arrays.toString(interp.getOutputTensor(0).shape());
        } catch (Exception e) {
            return modelName + ": LOAD FAILED " + e.getMessage();
        }

        ByteBuffer in = ByteBuffer.allocateDirect(inBytes).order(ByteOrder.nativeOrder());
        ByteBuffer out = ByteBuffer.allocateDirect(outBytes).order(ByteOrder.nativeOrder());

        // warmup - first few runs include lazy allocation
        try {
            for (int i = 0; i < 3; i++) {
                in.rewind();
                out.rewind();
                interp.run(in, out);
            }
        } catch (Exception e) {
            interp.close();
            return modelName + ": RUN FAILED " + e.getMessage();
        }

        long start = System.nanoTime();
        long deadline = start + RUN_SECONDS * 1_000_000_000L;
        long n = 0, total = 0;
        long firstTenTotal = 0, firstTenN = 0;
        long lastTenTotal = 0, lastTenN = 0;

        while (opModeIsActive() && System.nanoTime() < deadline) {
            in.rewind();
            out.rewind();
            long t0 = System.nanoTime();
            interp.run(in, out);
            long dt = System.nanoTime() - t0;

            total += dt;
            n++;

            long elapsed = t0 - start;
            if (elapsed < 10_000_000_000L) {
                firstTenTotal += dt;
                firstTenN++;
            }
            if (System.nanoTime() > deadline - 10_000_000_000L) {
                lastTenTotal += dt;
                lastTenN++;
            }

            if (n % 20 == 0) {
                telemetry.addData("running", modelName);
                telemetry.addData("shape", shapeInfo);
                telemetry.addData("mean ms", total / n / 1e6);
                telemetry.addData("fps", 1e9 * n / (double) (System.nanoTime() - start));
                telemetry.addData("elapsed s", (System.nanoTime() - start) / 1e9);
                for (String line : done) telemetry.addLine(line);
                telemetry.update();
            }
        }

        interp.close();

        double meanMs = total / (double) n / 1e6;
        double firstMs = firstTenN > 0 ? firstTenTotal / (double) firstTenN / 1e6 : 0;
        double lastMs = lastTenN > 0 ? lastTenTotal / (double) lastTenN / 1e6 : 0;

        return String.format("%s: %.0fms (%.1f fps) | first10s %.0f last10s %.0f | drift %+.0f%%",
                modelName, meanMs, 1000.0 / meanMs, firstMs, lastMs,
                firstMs > 0 ? (lastMs - firstMs) / firstMs * 100 : 0);
    }

    private MappedByteBuffer loadAsset(String name) throws IOException {
        try (AssetFileDescriptor fd = hardwareMap.appContext.getAssets().openFd(name);
             FileInputStream is = new FileInputStream(fd.getFileDescriptor())) {
            return is.getChannel().map(FileChannel.MapMode.READ_ONLY,
                    fd.getStartOffset(), fd.getDeclaredLength());
        }
    }
}