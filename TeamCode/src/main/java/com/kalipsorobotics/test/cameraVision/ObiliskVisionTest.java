package com.kalipsorobotics.test.cameraVision;

import android.util.Log;

import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.cameraVision.MotifColor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Obelisk Vision Test")
public class ObiliskVisionTest extends LinearOpMode {
    
    private ObiliskDetection obeliskDetection;
    
    @Override
    public void runOpMode() throws InterruptedException {
        obeliskDetection = new ObiliskDetection();
        obeliskDetection.init(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "camera1");
        telemetry.addData("Instructions", "Start to begin Obelisk detection");
        telemetry.addData("Controls", "DPAD_DOWN: Stop streaming, DPAD_UP: Resume streaming");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                
                obeliskDetection.addTelemetry(telemetry);
                
                if (obeliskDetection.isObeliskVisible()) {
                    telemetry.addLine("\n--- OBELISK DETECTED ---");
                    
                    String pattern = obeliskDetection.getMotifPatternString();
                    telemetry.addData("Pattern", pattern);
                    
                    // Simple ID-based pattern detection
                    int obeliskId = obeliskDetection.getObeliskId();
                    
                    if (obeliskId == 21) {
                        Log.d("obeliskDetection", "Green-Purple-Purple🟩🟪🟪");
                        telemetry.addData("PATTERN MATCH", "Green-Purple-Purple 🟩🟪🟪");
                    } else if (obeliskId == 22) {
                        Log.d("obeliskDetection", "Purple-Green-Purple🟪🟩🟪");
                        telemetry.addData("PATTERN MATCH", "Purple-Green-Purple 🟪🟩🟪");
                    } else if (obeliskId == 23) {
                        Log.d("obeliskDetection", "Purple-Purple-Green🟪🟪🟩");
                        telemetry.addData("PATTERN MATCH", "Purple-Purple-Green 🟪🟪🟩");
                    } else {
                        Log.d("obeliskDetection", "Unknown AprilTag ID: " + obeliskId);
                        telemetry.addData("RESULT", "Unknown AprilTag ID: " + obeliskId);
                    }
                } else {
                    Log.d("obeliskDetection", "No Obelisk detected");
                }
                
                if (gamepad1.dpad_down) {
                    obeliskDetection.stopStreaming();
                    Log.d("obeliskDetection", "Streaming STOPPED");
                } else if (gamepad1.dpad_up) {
                    obeliskDetection.resumeStreaming();
                    Log.d("obeliskDetection", "Streaming RESUMED");
                }
                
                telemetry.update();
                sleep(20);
            }
        }
        
        obeliskDetection.close();
    }
}
