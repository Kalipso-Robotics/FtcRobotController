package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.actions.outtake.PurePursuitDistanceSensorCorrection;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class WallToBarHangAction extends KActionSet {

    public WallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int barY) {
        this(driveTrain, wheelOdometry, outtake, barY, false, true);
    }

    public WallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int barY, boolean firstAction) {
        this(driveTrain, wheelOdometry, outtake, barY, false, false);
    }


    public WallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int barY,
                               boolean isSlow, boolean firstAction) {

        WaitAction waitAtStart = new WaitAction(50);
        waitAtStart.setName("waitAtStart");
        this.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        if(firstAction) {
            moveToSpecimenBar.addPoint(0, 0, 0);
        }
            moveToSpecimenBar.addPoint(-450, barY/2.0 + 50, 0,
                    isSlow?PurePursuitAction.P_XY : PurePursuitAction.P_XY_FAST,
                    PurePursuitAction.P_ANGLE);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, barY, 0, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        this.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake); // extra hang = -45
        specimenHangReady1.setName("hangSpecimenReady1");
        this.addAction(specimenHangReady1);

//        PurePursuitDistanceSensorCorrection purePursuitDistanceSensorCorrection =
//                new PurePursuitDistanceSensorCorrection(outtake, moveToSpecimenBar, driveTrain);
//        purePursuitDistanceSensorCorrection.setName("barHangPickupDistanceSensorActionMoveUntilThreshold");
//        purePursuitDistanceSensorCorrection.setDependentActions(specimenHangReady1);
//        this.addAction(purePursuitDistanceSensorCorrection);

        DistanceDetectionAction distanceDetectionAction = new DistanceDetectionAction(outtake.getRevDistanceBottom(),
                155);
        distanceDetectionAction.setName("distanceDetectionAction");
        distanceDetectionAction.setDependentActions(moveToSpecimenBar);
        this.addAction(distanceDetectionAction);

        MoveToDistanceThreshold moveToDistanceThreshold = new MoveToDistanceThreshold(driveTrain,
                distanceDetectionAction, -0.3);
        moveToDistanceThreshold.setName("moveToDistanceThreshold");
        moveToDistanceThreshold.setDependentActions(moveToSpecimenBar);
        this.addAction(moveToDistanceThreshold);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(specimenHangReady1, moveToDistanceThreshold);
        this.addAction(specimenHang);

    }
}
