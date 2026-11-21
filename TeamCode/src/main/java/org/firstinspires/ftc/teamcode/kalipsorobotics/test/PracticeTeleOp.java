package org.firstinspires.ftc.teamcode.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.DriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.Odometry;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KGamePad;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;
@TeleOp
public class PracticeTeleOp extends KOpMode {
    DriveTrain driveTrain;
    DriveAction driveAction;
    Intake intake;
    Stopper stopper;
    @Override
    protected void initializeRobot() {
        super.initializeRobot();
        driveTrain = DriveTrain.getInstance(opModeUtilities);
        driveAction = new DriveAction(driveTrain);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        Odometry odometry = Odometry.getInstance(opModeUtilities,driveTrain,imuModule);
        OpModeUtilities.runOdometryExecutorService(odoExecutorService,odometry);
        intake = new Intake(opModeUtilities);
        stopper = new Stopper(opModeUtilities);
        FieldOrientedDriveAction fieldOrientedDriveAction = new FieldOrientedDriveAction(driveTrain, imuModule);

    }

    @Override
    protected void initializeRobotConfig() {
        super.initializeRobotConfig();
        this.allianceColor = AllianceColor.RED;
        SharedData.setAllianceColor(allianceColor);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        waitForStart();
        while(opModeIsActive()){
            if(kGamePad1.getLeftStickX()!=0||kGamePad1.getLeftStickY()!=0||kGamePad1.getRightStickX()!=0){
                driveAction.move(kGamePad1.getGamepad());
            }
            else{
                driveTrain.setPower(0.0);
            }
        }
        cleanupRobot();
    }
}
