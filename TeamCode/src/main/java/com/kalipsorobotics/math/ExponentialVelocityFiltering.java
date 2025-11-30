package com.kalipsorobotics.math;

public class ExponentialVelocityFiltering {
    private double alpha;
    private double filteredVelocity;
    private double pastVelocity;
    private boolean initialized;

    public ExponentialVelocityFiltering(double alpha) {
        if (alpha < 0.0 || alpha > 1.0) {
            throw new IllegalArgumentException("Alpha must be between 0.0 and 1.0");
        }
        this.alpha = alpha;
        this.pastVelocity = 0.0;
        this.filteredVelocity = 0.0;
        this.initialized = false;
    }

    public double calculateFilteredVelocity(double newVelocity) {
        // Handle invalid inputs
        if (Double.isNaN(newVelocity) || Double.isInfinite(newVelocity)) {
            return filteredVelocity;
        }

        // On first call, initialize with the first measurement
        if (!initialized) {
            filteredVelocity = newVelocity;
            pastVelocity = newVelocity;
            initialized = true;
            return filteredVelocity;
        }

        filteredVelocity = newVelocity * alpha + (1 - alpha) * pastVelocity;
        pastVelocity = filteredVelocity;
        return filteredVelocity;
    }

    public void reset() {
        filteredVelocity = 0.0;
        pastVelocity = 0.0;
        initialized = false;
    }

    public double getFilteredVelocity() {
        return filteredVelocity;
    }







}
