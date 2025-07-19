package com.kalipsorobotics;

import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X;
import static com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_Y;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;

public class WallToBarAction extends KActionSet {

    public WallToBarAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Position hangPosition) {
        this(driveTrain, wheelOdometry, hangPosition, 0);
    }

    //ASSUME ROBOT AT WALL READY FOR SPECIMEN
    public WallToBarAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Position hangPosition,
                           int hangIncrement) {
        PurePursuitAction moveToBar1 = new PurePursuitAction(driveTrain, wheelOdometry); // Chunking pure pursuit
        moveToBar1.setName("moveToBar1");
        moveToBar1.setMaxTimeOutMS(3500);
        moveToBar1.addPoint(SPECIMEN_HANG_POS_X + 250, SPECIMEN_HANG_POS_Y - 250, -90, PurePursuitAction.P_XY_FAST,
                PurePursuitAction.P_ANGLE_FAST);

        if (hangPosition == null) {
            moveToBar1.addPoint(-450, (225 + hangIncrement)/2.0, 0, PurePursuitAction.P_XY_FAST,
                    PurePursuitAction.P_ANGLE);
            moveToBar1.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, 225 + hangIncrement, 0,
                    PurePursuitAction.P_XY_SLOW,
                    PurePursuitAction.P_ANGLE);
        } else {
            moveToBar1.addPoint(hangPosition.getX(), hangPosition.getY() + 75, hangPosition.getTheta());
        }
        this.addAction(moveToBar1);
        //waits for everything to finish to prevent specimen from getting caught in bar
//        PurePursuitAction moveToBar2 = new PurePursuitAction(driveTrain, wheelOdometry,1.0/1500.0); // Final pure pursuit
//        moveToBar2.setName("moveToBar2");
//        moveToBar2.setMaxTimeOutMS(1000);
//        moveToBar2.setDependentActions(moveToBar1);
//        moveToBar2.addPoint(SPECIMEN_HANG_POS_X, hangPosY, 0);
//        this.addAction(moveToBar2);
    }
}
