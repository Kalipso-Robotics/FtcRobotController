package com.kalipsorobotics.math;

public class LimelightPos {

    private double camDistanceToGoalMM;
    private double camAngleToAprilTagRad;
    private double xMM;
    private double yMM;
    private double zMM;

    public LimelightPos(double camDistanceToAprilTagMM, double camAngleToAprilTag, double xMM, double yMM, double zMM) {
        this.camDistanceToGoalMM = camDistanceToAprilTagMM;
        this.camAngleToAprilTagRad = camAngleToAprilTag;
        this.xMM = xMM;
        this.yMM = yMM;
        this.zMM = zMM;
    }

    public void setCamDistanceToGoalMM(double camDistanceToGoalMM) {
        this.camDistanceToGoalMM = camDistanceToGoalMM;
    }

    public void setCamAngleToAprilTagRad(double camAngleToAprilTagRad) {
        this.camAngleToAprilTagRad = camAngleToAprilTagRad;
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

    public double getCamDistanceToGoalMM() {
        return camDistanceToGoalMM;
    }

    public double getCamAngleToAprilTagRad() {
        return camAngleToAprilTagRad;
    }

    public boolean isEmpty() {
        if (getxMM() == 0 && getyMM() == 0 && getzMM() == 0 && getCamAngleToAprilTagRad() == 0) {
            return true;
        }
        return false;
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
        this.camAngleToAprilTagRad = position.getCamAngleToAprilTagRad();
        this.camDistanceToGoalMM = position.getCamDistanceToGoalMM();
    }

    public void reset() {
        this.camAngleToAprilTagRad = 0;
        this.camDistanceToGoalMM = 0;
        this.zMM = 0;
        this.xMM = 0;
        this.yMM = 0;
    }

    @Override
    public String toString() {
        return "LimelightPos{" +
                "distanceToTargetMM=" + camDistanceToGoalMM +
                ", angleToGoalRad=" + camAngleToAprilTagRad +
                ", xMM=" + xMM +
                ", yMM=" + yMM +
                ", zMM=" + zMM +
                '}';
    }
}

