package com.kalipsorobotics.math;

public class ExponentialVelocityFiltering {
    private final double alpha;
    private Velocity filteredVelocity;
    private Velocity pastVelocity;
    private boolean initialized;

    public ExponentialVelocityFiltering(double alpha) {
        if (alpha < 0.0 || alpha > 1.0) {
            throw new IllegalArgumentException("Alpha must be between 0.0 and 1.0");
        }
        this.alpha = alpha;
        this.pastVelocity = new Velocity(0,0,0);
        this.filteredVelocity = new Velocity(0,0,0);
        this.initialized = false;
    }

    public Velocity calculateFilteredVelocity(Velocity newVelocity) {

        if (!initialized) {
            filteredVelocity = newVelocity;
            pastVelocity = newVelocity;
            initialized = true;
            return filteredVelocity;
        }

        double filteredX = newVelocity.getX() * alpha + (1 - alpha) * pastVelocity.getX();
        double filteredY = newVelocity.getY() * alpha + (1 - alpha) * pastVelocity.getY();
        double filteredTheta = newVelocity.getTheta() * alpha + (1 - alpha) * pastVelocity.getTheta();

        filteredVelocity = new Velocity(filteredX, filteredY, filteredTheta);

        pastVelocity = filteredVelocity;
        return filteredVelocity;
    }

    public void reset() {
        filteredVelocity = new Velocity(0,0,0);
        pastVelocity = new Velocity(0,0,0);
        initialized = false;
    }

    public Velocity getFilteredVelocity() {
        return filteredVelocity;
    }







}
