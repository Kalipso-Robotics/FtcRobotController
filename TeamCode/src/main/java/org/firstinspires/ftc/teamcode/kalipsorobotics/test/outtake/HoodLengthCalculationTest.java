package org.firstinspires.ftc.teamcode.kalipsorobotics.test.outtake;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.CalculateHoodLength;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.ShooterLUTCornerGrid;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.GoBildaOdoModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class HoodLengthCalculationTest extends LinearOpMode {
    CalculateHoodLength calculateHoodLength;
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        GoBildaOdoModule goBildaOdoModule = new GoBildaOdoModule(opModeUtilities);
        Odometry odometry = Odometry.getInstance(opModeUtilities, driveTrain, imuModule);
        calculateHoodLength = new CalculateHoodLength(new Position(0, 0, 0)); //TODO find actual goal position
        ShooterLUTCornerGrid lut = new ShooterLUTCornerGrid();

        double xIn = 1000*(SharedData.getOdometryWheelIMUPosition().getX()); //converts to meters
        double yIn = 1000*(SharedData.getOdometryWheelIMUPosition().getY());


        ExecutorService executorService = Executors.newSingleThreadExecutor();
        OpModeUtilities.runOdometryExecutorService(executorService, odometry);
        waitForStart();
        while(opModeIsActive()) {
            //TODO make into action so it dosnt pause other things
            ShooterLUTCornerGrid.ShotParams sp = lut.queryInches(xIn, yIn);
            KLog.d("Hood Length Calculation", () -> "hood length: " + sp.thetaDeg + "Motor rpm: " + sp.rpm);

            //replce with  real hood and flywheel motor
            //hood.setAngleDegrees(sp.thetaDeg);
            //flywheel.setTargetRPM(sp.rpm);
        }
    }
}
