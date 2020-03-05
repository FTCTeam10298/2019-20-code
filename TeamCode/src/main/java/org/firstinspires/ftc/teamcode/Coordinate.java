package org.firstinspires.ftc.teamcode;

/**
 * Holds an x-y coordinate with angle. All measurements are in inches and all angles are in radians.
 * In our coordinate system, (0,0,0) is on center of the the wall closest to the audience facing the
 * right. +X is towards the right, +Y is away from the audience, and +theta is counter-clockwise.
 */

public class Coordinate {

    private double x;
    private double y;
    private double angle;

    /**
     * Creates a Coordinate with given values. All alternate constructors assume 0 for unstated variables.
     * @param xPos X position
     * @param yPos Y position
     * @param anglePos Angle
     */
    public Coordinate (double xPos, double yPos, double anglePos) {
        x = xPos;
        y = yPos;
        angle = Math.toRadians(anglePos);
    }

    public Coordinate (double xPos, double yPos) {
        this(xPos, yPos, 0);
    }

    public Coordinate (double anglePos) {
        this(0, 0, anglePos);
    }

    public Coordinate () {
        this(0, 0, 0);
    }

    /**
     * Returns the x of the Coordinate
     * @return X
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y of the Coordinate
     * @return Y
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the angle of the Coordinate
     * @return Angle
     */
    public double getAngle() {
        return angle;
    }

    /**
     * Sets the x component of the Coordinate in inches.
     * @param x The x value that we want to set.
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Sets the y component of the Coordinate in inches.
     * @param y The y value that we want to set.
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Sets the angle of the Coordinate in radians.
     * @param angle The angle that we want to set.
     */
    public void setAngle(double angle) {
        this.angle = angle;
    }

    /**
     * Sets all of the parameters of the Coordinate.
     * @param x The x value that we want to set.
     * @param y The y value that we want to set.
     * @param angle
     */
    public void setCoordinate(double x, double y, double angle) {
        this.setX(x);
        this.setY(y);
        this.setAngle(Math.toRadians(angle));
    }

    /**
     * Wraps the angle around so that the robot doesn't unnecessarily turn over 180 degrees.
     * @param angle The angle to wrap.
     * @return The wrapped angle.
     */
    public double wrapAngle(double angle) {
        return angle % (2*Math.PI);
    }

    /**
     * Gives the absolute value of the distance between the given Coordinate and the current Coordinate.
     * @param coordinate Coordinate to compare
     * @return distance from current Coordinate
     */
    public double distance (Coordinate coordinate) {
        return Math.hypot(coordinate.getX() - x, coordinate.getY() - y);
    }

    /**
     * Gives the absolute value of the distance between the X and Y values and the current Coordinate.
     * @param targetX X
     * @param targetY Y
     * @return distance from current Coordinate
     */
    public double distance (double targetX, double targetY) {
        return Math.hypot(targetX - x, targetY - y);
    }

    /**
     * Gives the error of the angle from the given Coordinate and the current Coordinate.
     * @param coordinate Coordinate to compare
     * @return angle error from current Coordinate
     */
    public double theta (Coordinate coordinate) {
        return wrapAngle(coordinate.getAngle() - angle);
    }

    /**
     * Gives the error of the angle from the given angle and the current Coordinate.
     * @param targetA angle to compare
     * @return angle error from current Coordinate
     */
    public double theta (double targetA) {
        return  wrapAngle(targetA - angle);
    }

    @Override
    public boolean equals(Object obj) {
        return super.equals(obj);
    }

    public String toString () {
        return "X: " + x + "\nY: " + y + "\nAngle: " + angle;
    }
}
