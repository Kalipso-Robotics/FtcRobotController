package com.kalipsorobotics.math;

public class LimelightPos {

    private double apriLTagDistanceToCamMM;
    private double goalAngleToCamRad;
    private double xMM;
    private double yMM;
    private double zMM;

    public LimelightPos(double aprilTagDistanceToCamMM, double goalAngleToCamRad, double xMM, double yMM, double zMM) {
        this.apriLTagDistanceToCamMM = aprilTagDistanceToCamMM;
        this.goalAngleToCamRad = goalAngleToCamRad;
        this.xMM = xMM;
        this.yMM = yMM;
        this.zMM = zMM;
    }

    public void setApriLTagDistanceToCamMM(double apriLTagDistanceToCamMM) {
        this.apriLTagDistanceToCamMM = apriLTagDistanceToCamMM;
    }

    public void setGoalAngleToCamRad(double goalAngleToCamRad) {
        this.goalAngleToCamRad = goalAngleToCamRad;
    }

    public void setxMM(double xMM) {
        this.xMM = xMM;
    }

    public void setyMM(double yMM) {
        this.yMM = yMM;
    }

    public void setzMM(double zMM) {
        this.zMM = zMM;
    }

    public double getApriLTagDistanceToCamMM() {
        return apriLTagDistanceToCamMM;
    }

    public double getGoalAngleToCamRad() {
        return goalAngleToCamRad;
    }

    public boolean isEmpty() {
        return getxMM() == 0 && getyMM() == 0 && getzMM() == 0 && getGoalAngleToCamRad() == 0;
    }

    public double getxMM() {
        return xMM;
    }

    public double getyMM() {
        return yMM;
    }

    public double getzMM() {
        return zMM;
    }

    public void setPos(LimelightPos position) {
        this.xMM = position.getxMM();
        this.yMM = position.getyMM();
        this.zMM = position.getzMM();
        this.goalAngleToCamRad = position.getGoalAngleToCamRad();
        this.apriLTagDistanceToCamMM = position.getApriLTagDistanceToCamMM();
    }

    public void reset() {
        this.goalAngleToCamRad = 0;
        this.apriLTagDistanceToCamMM = 0;
        this.zMM = 0;
        this.xMM = 0;
        this.yMM = 0;
    }

    @Override
    public String toString() {
        return "LimelightPos{" +
                "distanceToTargetMM=" + apriLTagDistanceToCamMM +
                ", angleToGoalRad=" + goalAngleToCamRad +
                ", xMM=" + xMM +
                ", yMM=" + yMM +
                ", zMM=" + zMM +
                '}';
    }
}

