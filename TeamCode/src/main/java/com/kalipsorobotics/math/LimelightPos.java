package com.kalipsorobotics.math;

public class LimelightPos {

    private double distanceToAprilTagMM;
    private double angleToGoalRad;
    private double xMM;
    private double yMM;
    private double zMM;

    public LimelightPos(double distanceToAprilTagMM, double angleToGoalRad, double xMM, double yMM, double zMM) {
        this.distanceToAprilTagMM = distanceToAprilTagMM;
        this.angleToGoalRad = angleToGoalRad;
        this.xMM = xMM;
        this.yMM = yMM;
        this.zMM = zMM;
    }

    public void setDistanceToAprilTagMM(double distanceToAprilTagMM) {
        this.distanceToAprilTagMM = distanceToAprilTagMM;
    }

    public void setAngleToGoalRad(double angleToGoalRad) {
        this.angleToGoalRad = angleToGoalRad;
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

    public double getDistanceToAprilTagMM() {
        return distanceToAprilTagMM;
    }

    public double getAngleToGoalRad() {
        return angleToGoalRad;
    }

    public boolean isEmpty() {
        if (getxMM() == 0 && getyMM() == 0 && getzMM() == 0 && getAngleToGoalRad() == 0) {
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
        this.angleToGoalRad = position.getAngleToGoalRad();
        this.distanceToAprilTagMM = position.getDistanceToAprilTagMM();
    }

    public void reset() {
        this.angleToGoalRad = 0;
        this.distanceToAprilTagMM = 0;
        this.zMM = 0;
        this.xMM = 0;
        this.yMM = 0;
    }

    @Override
    public String toString() {
        return "LimelightPos{" +
                "distanceToTargetMM=" + distanceToAprilTagMM +
                ", angleToGoalRad=" + angleToGoalRad +
                ", xMM=" + xMM +
                ", yMM=" + yMM +
                ", zMM=" + zMM +
                '}';
    }
}

