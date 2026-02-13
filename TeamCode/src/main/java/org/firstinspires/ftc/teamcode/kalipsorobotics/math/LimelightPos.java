package org.firstinspires.ftc.teamcode.kalipsorobotics.math;

public class LimelightPos {

    private double aprilTagDistanceToCamMM;
    private double goalAngleToCamRad;
    private double xMM;
    private double yMM;
    private double zMM;

    public LimelightPos(double aprilTagDistanceToCamMM, double goalAngleToCamRad, double xMM, double yMM, double zMM) {
        this.aprilTagDistanceToCamMM = aprilTagDistanceToCamMM;
        this.goalAngleToCamRad = goalAngleToCamRad;
        this.xMM = xMM;
        this.yMM = yMM;
        this.zMM = zMM;
    }

    public void setAprilTagDistanceToCamMM(double aprilTagDistanceToCamMM) {
        this.aprilTagDistanceToCamMM = aprilTagDistanceToCamMM;
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

    public double getAprilTagDistanceToCamMM() {
        return aprilTagDistanceToCamMM;
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
        this.aprilTagDistanceToCamMM = position.getAprilTagDistanceToCamMM();
    }

    public void reset() {
        this.goalAngleToCamRad = 0;
        this.aprilTagDistanceToCamMM = 0;
        this.zMM = 0;
        this.xMM = 0;
        this.yMM = 0;
    }

    @Override
    public String toString() {
        return "LimelightPos{" +
                "distanceToTargetMM=" + aprilTagDistanceToCamMM +
                ", angleToGoalRad=" + goalAngleToCamRad +
                ", xMM=" + xMM +
                ", yMM=" + yMM +
                ", zMM=" + zMM +
                '}';
    }
}

