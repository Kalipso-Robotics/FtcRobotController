package com.kalipsorobotics.test.outtake;

import com.kalipsorobotics.utilities.KLog;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.CalculateHoodLength;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.ShooterLUTCornerGrid;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.GoBildaOdoModule;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
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
            KLog.d("Hood Length Calculation", "hood length: " + sp.thetaDeg + "Motor rpm: " + sp.rpm);

            //replce with  real hood and flywheel motor
            //hood.setAngleDegrees(sp.thetaDeg);
            //flywheel.setTargetRPM(sp.rpm);
        }
    }
}
