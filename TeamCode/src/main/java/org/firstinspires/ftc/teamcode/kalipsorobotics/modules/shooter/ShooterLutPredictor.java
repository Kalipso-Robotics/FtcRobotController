package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import android.content.Context;
import java.io.IOException;

/**
 * Adapter that wraps ShooterLutPredictor to implement IShooterPredictor interface.
 * Converts distance in millimeters to pixel coordinates for the LUT predictor.
 */
public class ShooterLutPredictor implements IShooterPredictor {
    private final ShooterLutPredictorInternal predictor;

    /**
     * Create adapter with LUT predictor loaded from assets.
     * @param ctx Android context
     * @param assetName Name of the LUT binary file
     * @throws IOException if file cannot be loaded
     */
    public ShooterLutPredictor(Context ctx, String assetName) throws IOException {
        this.predictor = new ShooterLutPredictorInternal(ctx, assetName);
    }

    /**
     * Get shooter parameters for a given distance using LUT interpolation.
     *
     * @param distanceMM Distance to target in millimeters
     * @return Shooter parameters (RPS and hood position)
     */
    @Override
    public IShooterPredictor.ShooterParams predict(double distanceMM) {
        // Convert millimeters to pixels
        double xPixels = Shooter.mmToPixel(distanceMM);
        double yPixels = Shooter.mmToPixel(Shooter.GOAL_HEIGHT_MM);

        // Get prediction from LUT
        ShooterLutPredictorInternal.Prediction prediction = predictor.predict(xPixels, yPixels);

        // Convert to common interface type
        // Note: HOOD_OFFSET is added here to match ShooterManualDataLookup behavior
        return new IShooterPredictor.ShooterParams(
            prediction.rps,
            prediction.hood + Shooter.HOOD_OFFSET
        );
    }
}
