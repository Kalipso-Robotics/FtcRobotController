package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher;


import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KServoAutoAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.RunIntakeTime;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class PushBall extends KActionSet {
//    private final RunIntakeUntilFullSpeed runUntilFullSpeed;
    public PushBall(Stopper stopper, Intake intake) {

        KLog.d("PushAllBalls", "current pos " + SharedData.getOdometryWheelIMUPosition());

        KServoAutoAction openStopper = new KServoAutoAction(stopper.getStopper(), ModuleConfig.STOPPER_SERVO_OPEN_POS);
        openStopper.setName("openStopper");
        this.addAction(openStopper);

        KLog.d("PushAllBalls", "openStopper " + openStopper.getIsDone());

        RunIntakeTime runIntakeTime = new RunIntakeTime(intake, IntakeConfig.shootTimeMS, IntakeConfig.intakePower);
        runIntakeTime.setName("runIntakeTime");
        runIntakeTime.setDependentActions(openStopper);
        this.addAction(runIntakeTime);
//
//        runUntilFullSpeed = new RunIntakeUntilFullSpeed(intake);
//        runUntilFullSpeed.setDependentActions(openStopper);
//        runUntilFullSpeed.setName("untilShootingDone");
////        untilShootingDone.setDependentActions(pushLeft, pushRight);
//        this.addAction(runUntilFullSpeed);
//
//        KLog.d("PushAllBalls", "run intake " + runUntilFullSpeed.getIsDone());


        KLog.d("teleop", "pusher Stopped(based off intake)");

    }

//    public RunIntakeUntilFullSpeed getRunUntilFullSpeed() {
//        return runUntilFullSpeed;
//    }

}
