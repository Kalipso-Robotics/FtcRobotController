package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

/**
 * Interface for shooter parameter prediction.
 * Implementations provide RPS and hood position based on distance to target.
 */
public interface IShooterPredictor {

    /**
     * Result containing shooter parameters.
     */
    class ShooterParams {
        public final double shooterRPS;
        public final double hoodPosition;
        public final double intakeRPS;

        public ShooterParams(double shooterRPS, double hoodPosition, double intakeRPS) {
            this.shooterRPS = shooterRPS;
            this.hoodPosition = hoodPosition;
            this.intakeRPS = intakeRPS;
        }

        @Override
        public String toString() {
            return String.format("ShooterParams(rps=%.2f, hood=%.4f)", shooterRPS, hoodPosition);
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
