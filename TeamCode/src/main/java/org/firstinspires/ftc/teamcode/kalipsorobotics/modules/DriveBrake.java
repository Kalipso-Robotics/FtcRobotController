package org.firstinspires.ftc.teamcode.kalipsorobotics.modules;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig.*;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KServo;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

public class DriveBrake {

    private final KServo brake;
    //C2 brakeRight  C5 brakeLeft
    public DriveBrake(OpModeUtilities opModeUtilities) {
        this.brake = new KServo(opModeUtilities.getHardwareMap().servo.get("brake")); // port 2
        init();
    }

    private void init() {
        this.brake.setTargetPosition(RELEASE_BRAKE_POS);
    }


    public KServo getBrake() {
        return brake;
    }
}
