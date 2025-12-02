package com.kalipsorobotics.utilities;

import com.kalipsorobotics.cameraVision.AllianceColor;
import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.PositionHistory;

import java.util.HashMap;

public class SharedData {

    private static final Position odometryPosition = new Position(0, 0, 0);

    private static final HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionMap = new HashMap<>();

    public static Position getOdometryPosition() {
        return new Position(odometryPosition);
    }

    public static void setOdometryPosition(Position position) {
        odometryPosition.reset(position);
    }

    public static void resetOdometryPosition() {
        odometryPosition.reset(new Position(0, 0, 0));
    }

    public static HashMap<OdometrySensorCombinations, PositionHistory> getOdometryPositionMap() {
        return new HashMap<>(odometryPositionMap);
    }

    public static void setOdometryPositionMap(HashMap<OdometrySensorCombinations, PositionHistory> odometryPositionMap) {
        SharedData.odometryPositionMap.putAll(odometryPositionMap);
    }

    private static boolean isOdometryUnhealthy = false;

    public static boolean getOdometryUnhealthy() {
        return isOdometryUnhealthy;
    }

    public static void setIsOdometryUnhealthy(boolean isOdometryUnhealthy) {
        SharedData.isOdometryUnhealthy = isOdometryUnhealthy;
    }

    private static AllianceColor allianceColor = AllianceColor.RED;

    public static AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public static void setAllianceColor(AllianceColor allianceColor) {
        SharedData.allianceColor = allianceColor;
    }

}
