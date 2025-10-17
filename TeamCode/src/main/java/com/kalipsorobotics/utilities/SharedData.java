package com.kalipsorobotics.utilities;

import com.kalipsorobotics.cameraVision.AllianceSetup;
import com.kalipsorobotics.localization.OdometrySensorCombinations;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.PositionHistory;

import java.util.HashMap;

public class SharedData {

    private static final Position odometryPosition = new Position(0, 0, 0);

    public static boolean isIsTurretWithinRange() {
        return isTurretWithinRange;
    }

    public static void setIsTurretWithinRange(boolean isTurretWithinRange) {
        SharedData.isTurretWithinRange = isTurretWithinRange;
    }

    private static boolean isTurretWithinRange = false;
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

    private static AllianceSetup allianceSetup = AllianceSetup.RED;

    public static AllianceSetup getAllianceColor() {
        return allianceSetup;
    }

    public static void setAllianceColor(AllianceSetup allianceSetup) {
        SharedData.allianceSetup = allianceSetup;
    }
}
