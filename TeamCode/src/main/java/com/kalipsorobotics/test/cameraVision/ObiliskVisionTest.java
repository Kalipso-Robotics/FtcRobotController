package com.kalipsorobotics.test.cameraVision;

import android.util.Log;

import com.kalipsorobotics.cameraVision.ObiliskDetection;
import com.kalipsorobotics.cameraVision.ObiliskDetection.MotifColor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Obelisk Vision Test", group = "Test")
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
                    
                    if (obeliskDetection.isMotifPattern(MotifColor.PURPLE, MotifColor.PURPLE, MotifColor.GREEN)) {
                        Log.d("obeliskDetection", "Purple-Purple-Green🟪🟪🟩");
                    } else if (obeliskDetection.isMotifPattern(MotifColor.PURPLE, MotifColor.GREEN, MotifColor.PURPLE)) {
                        Log.d("obeliskDetection", "Purple-Green-Purple🟪🟩🟪");
                    } else if (obeliskDetection.isMotifPattern(MotifColor.GREEN, MotifColor.PURPLE, MotifColor.PURPLE)) {
                        Log.d("obeliskDetection", "Green-Purple-Purple🟩🟪🟪");
                    } else {
                        Log.d("obeliskDetection", "Unknown or partial pattern");
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
