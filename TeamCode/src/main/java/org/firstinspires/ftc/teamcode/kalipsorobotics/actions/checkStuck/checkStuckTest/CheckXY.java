package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.checkStuck.checkStuckTest;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Path;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

public class CheckXY {
    OpModeUtilities opModeUtilities;
    double MIN_POSITION_THRESHOLD;
    public CheckXY(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        MIN_POSITION_THRESHOLD = 200; //replace with something accurate
        //in mm
    }
    public boolean isPositionOnPath(Path happyPath, int timeInMs) {
        int extra = timeInMs%1000;
        //path with increments each second
        Position intendedPos = happyPath.getPoint((timeInMs-extra)/1000);
        Position currentPos = SharedData.getOdometryWheelIMUPosition();
        return abs(intendedPos.getX() - currentPos.getX()) < MIN_POSITION_THRESHOLD && abs(intendedPos.getY() - currentPos.getY()) < MIN_POSITION_THRESHOLD; // Robot IS on path
// Robot is NOT on path
    }
}
