package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.kalipsorobotics.cameraVision.AllianceColor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.OdometrySensorCombinations;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.LimelightPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.PositionHistory;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Velocity;

import java.util.HashMap;

public class SensorData {

    private static double intakeMotorTicks = 0;

    public static double getIntakeMotorTicks() {
        return intakeMotorTicks;
    }

    public static void setIntakeMotorTicks(double intakeMotorTicks) {
        SensorData.intakeMotorTicks = intakeMotorTicks;
    }

    private static double turretMotorTicks = 0;

    public static double getTurretMotorTicks() {
        return turretMotorTicks;
    }
    public static void setTurretMotorTicks(double turretMotorTicks) {
        SensorData.turretMotorTicks = turretMotorTicks;
    }

    private static double shooterMotorTicks = 0;

    public static double getShooterMotorTicks() {
        return shooterMotorTicks;
    }
    public static void setShooterMotorTicks (double shooterMotorTicks) {
        SensorData.shooterMotorTicks = shooterMotorTicks;
    }

    //sensors
    private static double distanceSensorReading = 0;

    public static double getDistanceSensorReading() {
        return distanceSensorReading;
    }
    public static void setDistanceSensorReading(double distanceSensorReading) {
        SensorData.distanceSensorReading = distanceSensorReading;
    }

    private static NormalizedRGBA colorSensorReading = new NormalizedRGBA();

    public static NormalizedRGBA getColorSensorReading() {
        return colorSensorReading;
    }

    public static void setColorSensorReading(NormalizedRGBA colorSensorReading) {
        SensorData.colorSensorReading = colorSensorReading;
    }

}