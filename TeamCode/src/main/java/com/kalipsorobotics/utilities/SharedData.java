package com.kalipsorobotics.utilities;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.math.LimelightPos;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.PositionHistory;

import java.util.HashMap;

public class SharedData {

    private static final Position odometryIMUPosition = new Position(0, 0, 0);

    public static Position getOdometryWheelIMUPosition() {
        return new Position(odometryIMUPosition);
    }
    public static void setOdometryIMUPosition(Position position) {
        odometryIMUPosition.reset(position);
    }
    public static void resetOdometryIMUPosition() {
        odometryIMUPosition.reset(new Position(0, 0, 0));
    }


    private static final Position odometryWheelPosition = new Position(0, 0, 0);

    public static Position getOdometryWheelPosition() {
        return new Position(odometryWheelPosition);
    }
    public static void setOdometryWheelPosition(Position position) {
        odometryWheelPosition.reset(position);
    }
    public static void resetOdometryWheelPosition() {
        odometryWheelPosition.reset(new Position(0, 0, 0));
    }



    private static final HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionMap = new HashMap<>();
    public static HashMap<OdometrySensorCombinations, PositionHistory> getOdometryPositionMap() {
        return new HashMap<>(odometryPositionMap);
    }
    public static void setOdometryPositionMap(HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionMap) {
        SharedData.odometryPositionMap.putAll(odometryPositionMap);
    }


    private static final LimelightPos limelightPosition = new LimelightPos(0,0,0,0,0);
    public static void setLimelightPosition(LimelightPos position) {
        limelightPosition.setPos(position);
    }
    public static LimelightPos getLimelightPosition() {
        return limelightPosition;
    }

    private static final Position limelightGlobalPosition = new Position(0, 0, 0);
    public static Position getLimelightGlobalPosition() {
        return new Position(limelightGlobalPosition);
    }
    public static void setLimelightGlobalPosition(Position position) {
        limelightGlobalPosition.reset(position);
    }


    private static long unhealthyCounter = 0;
    public static long getUnhealthyCounter() {
        return unhealthyCounter;
    }
    public static void setUnhealthyCounter(long unhealthyCounter) {
        SharedData.unhealthyCounter = unhealthyCounter;
    }



    private static AllianceColor allianceColor = AllianceColor.RED;

    public static AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public static void setAllianceColor(AllianceColor allianceColor) {
        SharedData.allianceColor = allianceColor;
    }

}
