package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometrySensorCombinations;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.LimelightPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.PositionHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;

import java.util.HashMap;

public class SharedData {

    private static final Position odometryWheelIMUPosition = new Position(0, 0, 0);

    /** Returns a defensive copy. Safe for storing or mutating. */
    public static Position getOdometryWheelIMUPosition() {
        return new Position(odometryWheelIMUPosition);
    }

    /** Returns a direct reference - zero allocation. Use only for read-only access within a single loop iteration. */
    public static Position peekOdometryWheelIMUPosition() {
        return odometryWheelIMUPosition;
    }
    public static void setOdometryWheelIMUPosition(Position position) {
        odometryWheelIMUPosition.reset(position);
    }
    public static void resetOdometryWheelIMUPosition() {
        odometryWheelIMUPosition.reset(new Position(0, 0, 0));
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


    private static final LimelightPos limelightRawPosition = new LimelightPos(0,0,0,0,0);
    public static void setLimelightRawPosition(LimelightPos position) {
        limelightRawPosition.setPos(position);
    }
    public static LimelightPos getLimelightRawPosition() {
        return limelightRawPosition;
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


    //For transfer between Auto and TeleOp
    private static AllianceColor allianceColor = AllianceColor.RED;

    public static AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public static void setAllianceColor(AllianceColor allianceColor) {
        SharedData.allianceColor = allianceColor;
    }

    private static final Velocity odometryWheelVelocity = new Velocity(0, 0, 0);

    public static Velocity getOdometryWheelVelocity() {
        return new Velocity(odometryWheelVelocity);
    }
    public static void setOdometryWheelVelocity(Velocity velocity) {
        odometryWheelVelocity.reset(velocity);
    }
    public static void resetOdometryWheelVelocity() {
        odometryWheelVelocity.reset(new Velocity(0, 0, 0));
    }

    private static final Velocity odometryWheelIMUVelocity = new Velocity(0, 0, 0);

    /** Returns a defensive copy. Safe for storing or mutating. */
    public static Velocity getOdometryWheelIMUVelocity() {
        return new Velocity(odometryWheelIMUVelocity);
    }

    /** Returns a direct reference - zero allocation. Use only for read-only access within a single loop iteration. */
    public static Velocity peekOdometryWheelIMUVelocity() {
        return odometryWheelIMUVelocity;
    }
    public static void setOdometryWheelIMUVelocity(Velocity velocity) {
        odometryWheelIMUVelocity.reset(velocity);
    }
    public static void resetOdometryWheelIMUVelocity() {
        odometryWheelIMUVelocity.reset(new Velocity(0, 0, 0));
    }

    private static double voltage = ShooterInterpolationConfig.DEFAULT_VOLTAGE;
    public static double getVoltage() {
        return voltage;
    }
    public static void setVoltage(double newVoltage) {
        voltage = newVoltage;
    }

    private static Position unfilteredLimelightGlobalPos = new Position(0,0,0);
    public static Position getUnfilteredLimelightGlobalPos() {
        return new Position(unfilteredLimelightGlobalPos);
    }

    public static void setUnfilteredLimelightGlobalPos(Position position) {
        if (position != null) {
            unfilteredLimelightGlobalPos.reset(position);
        } else {
            unfilteredLimelightGlobalPos.reset(new Position(0,0,0));
        }
    }


}
