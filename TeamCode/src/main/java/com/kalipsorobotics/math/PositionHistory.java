package com.kalipsorobotics.math;

public class PositionHistory {

    private Position currentPosition;
    private Velocity currentVelocity;
    private Velocity relativeDelta;
    private int leftTicks;
    private int rightTicks;
    private int backTicks;
    private double leftDistanceMM;
    private double rightDistanceMM;
    private double backDistanceMM;

    public double getRawIMU() {
        return rawIMU;
    }

    public void setRawIMU(double rawIMU) {
        this.rawIMU = rawIMU;
    }

    private double rawIMU;


    public double getBackDistanceMM() {
        return backDistanceMM;
    }

    public double getRightDistanceMM() {
        return rightDistanceMM;
    }

    public double getLeftDistanceMM() {
        return leftDistanceMM;
    }


    public PositionHistory() {}

    public Position getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(Position currentPosition) {
        this.currentPosition = currentPosition;
    }

    public Velocity getCurrentVelocity() {
        return currentVelocity;
    }

    public void setCurrentVelocity(Velocity relativeDelta, double timeElapsed) {
        this.relativeDelta = relativeDelta;
        Velocity currentVelocity = this.relativeDelta.divide(timeElapsed);
        this.currentVelocity = currentVelocity;
    }

    public void setEncoderTicks(int leftTicks, int rightTicks, int backTicks) {
        this.leftTicks = leftTicks;
        this.rightTicks = rightTicks;
        this.backTicks = backTicks;
    }

    public void setDistanceMM(double leftDistanceMM, double rightDistanceMM, double backDistanceMM) {
        this.leftDistanceMM = leftDistanceMM;
        this.rightDistanceMM = rightDistanceMM;
        this.backDistanceMM = backDistanceMM;
    }

    public int getLeftTicks() {
        return leftTicks;
    }

    public int getRightTicks() {
        return rightTicks;
    }

    public int getBackTicks() {
        return backTicks;
    }

    public Velocity getRelativeDelta() {
        return relativeDelta;
    }

    public static String toNullString() {
        return "null, null, null, null, null, null, null";
    }

    public String toStringCSV() {
        return currentPosition.getX() + ", " +
                currentPosition.getY() + ", " +
                currentPosition.getTheta() + ", " +
                relativeDelta.getTheta() + ", " +
                leftTicks + ", " +
                rightTicks + ", " +
                backTicks;

    }

    @Override
    public String toString() {
        return "PositionHistory{" +
                "currentPosition=" + currentPosition +
                ", currentVelocity=" + currentVelocity +
                '}';
    }
}
