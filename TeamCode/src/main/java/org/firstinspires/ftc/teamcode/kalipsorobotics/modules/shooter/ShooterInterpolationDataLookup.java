package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig.FEED_RPS;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig.*;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.ArrayList;

/**
 * Linear interpolation lookup for shooter parameters based on manually tuned test data.
 * Rows live in ShooterInterpolationConfig.shooterData.
 */
public class ShooterInterpolationDataLookup implements IShooterPredictor {
    // No clamping - use the exact values from the manual tuning data

    public static class DataPoint {
        final double distanceMM;
        final double shooterRPS;
        final double hoodPosition;
        final double intakeRPS;

        DataPoint(double distanceMM, double rps, double hoodPosition, double intakeRPS) {
            this.distanceMM = distanceMM;
            this.shooterRPS = rps;
            this.hoodPosition = hoodPosition;
            this.intakeRPS = intakeRPS;
        }
    }

    // Rows come from ShooterInterpolationConfig.shooterData (distanceMM, rps, hoodPosition).
    // Rebuilt by each constructor so a second lookup never duplicates rows.
    private final ArrayList<DataPoint> dataPoints = new ArrayList<>();
    private boolean emptyTableLogged = false;

    /**
     * Default constructor - copies the rows from ShooterInterpolationConfig.shooterData.
     */
    public ShooterInterpolationDataLookup() {
        for (int i = 0; i + 2 < shooterData.length; i += 3) {
            dataPoints.add(new DataPoint(shooterData[i], shooterData[i + 1], shooterData[i + 2], shooterData[i+3]));
        }
    }

    /**
     * Get shooter parameters for a given distance using linear interpolation.
     * Empty table: safe default. Below the first row: first row. Above the last row: last row.
     *
     * @param distanceMM Distance to target in millimeters
     * @return Shooter parameters (RPS and hood position)
     */
    @Override
    public IShooterPredictor.ShooterParams predict(double distanceMM) {
        if (dataPoints.isEmpty()) {
            if (!emptyTableLogged) {
                emptyTableLogged = true;
                KLog.e(Shooter.TAG, "event=emptyInterpolationTable rps=" + DEFAULT_RPS + " hood=" + MIN_HOOD);
            }
            return toParams(DEFAULT_RPS, MIN_HOOD, FEED_RPS);
        }

        DataPoint first = dataPoints.get(0);
        if (distanceMM <= first.distanceMM) {
            return toParams(first.shooterRPS, first.hoodPosition, first.intakeRPS);
        }

        DataPoint last = dataPoints.get(dataPoints.size() - 1);
        if (distanceMM >= last.distanceMM) {
            return toParams(last.shooterRPS, last.hoodPosition, last.intakeRPS);
        }

        // Find the two surrounding points
        for (int i = 0; i < dataPoints.size() - 1; i++) {
            DataPoint lower = dataPoints.get(i);
            DataPoint upper = dataPoints.get(i + 1);

            if (distanceMM >= lower.distanceMM && distanceMM <= upper.distanceMM) {
                // Linear interpolation
                double t = (distanceMM - lower.distanceMM) / (upper.distanceMM - lower.distanceMM);
                double interpolatedShooterRPS = lower.shooterRPS + t * (upper.shooterRPS - lower.shooterRPS);
                double interpolatedHood = lower.hoodPosition + t * (upper.hoodPosition - lower.hoodPosition);
                double interpolatedIntakeRPS = lower.intakeRPS + t * (upper.intakeRPS - lower.intakeRPS);
                return toParams(interpolatedShooterRPS, interpolatedHood, interpolatedIntakeRPS);
            }
        }

        // Only reached if the table is not sorted by distance
        return toParams(last.shooterRPS, last.hoodPosition, last.intakeRPS);
    }

    private IShooterPredictor.ShooterParams toParams(double shooterRPS, double hood, double intakeRPS) {
        double clampedHood = clampHood(hood) + HOOD_INIT_OFFSET;
        return new IShooterPredictor.ShooterParams(shooterRPS + rpsOffset, clampedHood + hoodOffset, intakeRPS);
    }

    /**
     * Clamp hood position to valid range.
     */
    private double clampHood(double hood) {
        return MathFunctions.clamp(hood, MIN_HOOD, MAX_HOOD); // MIN MAX HOOD SWITCHED WHILE FLIP DIRECTION BROKEN IN KSERVO
    }

    /**
     * Get the number of data points loaded.
     */
    public int getDataPointCount() {
        return dataPoints.size();
    }
}
