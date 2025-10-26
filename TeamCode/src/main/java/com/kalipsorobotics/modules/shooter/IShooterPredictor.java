package com.kalipsorobotics.modules.shooter;

/**
 * Interface for shooter parameter prediction.
 * Implementations provide RPS and hood position based on distance to target.
 */
public interface IShooterPredictor {

    /**
     * Result containing shooter parameters.
     */
    class ShooterParams {
        public final double rps;
        public final double hoodPosition;

        public ShooterParams(double rps, double hoodPosition) {
            this.rps = rps;
            this.hoodPosition = hoodPosition;
        }

        @Override
        public String toString() {
            return String.format("ShooterParams(rps=%.2f, hood=%.4f)", rps, hoodPosition);
        }
    }

    /**
     * Get shooter parameters for a given distance to target.
     *
     * @param distanceMM Distance to target in millimeters
     * @return Shooter parameters (RPS and hood position)
     */
    ShooterParams predict(double distanceMM);
}
