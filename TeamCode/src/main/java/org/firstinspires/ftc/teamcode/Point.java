package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public abstract class Point {

    // Index 0: x coordinate
    // Index 1: y coordinate
    // Index 2: angle (degrees)
    private ArrayList<Double> point = new ArrayList<Double>();
    private static final double MINIMUM_DISTANCE_AWAY = 1;

    public Point () {
        this (0, 0, 0);
    }

    public Point (double angleDegrees) {
        this (0, 0, angleDegrees);
    }

    public Point (double xCoordinate, double yCoordinate) {
        this (xCoordinate, yCoordinate, 0);
    }

    public Point (double xCoordinate, double yCoordinate, double angleDegrees) {
        point.add(xCoordinate);
        point.add(yCoordinate);
        if (Math.abs(angleDegrees) <= 180)
            point.add(angleDegrees);
        else if (angleDegrees > 0)
            point.add(angleDegrees - 360);
        else
            point.add(angleDegrees + 360);
    }

    public ArrayList<Double> getPoint() {
        return point;
    }

    public double getX () {
        return point.get(0);
    }

    public double getY () {
        return point.get(1);
    }

    public double getAngle () {
        return point.get(2);
    }

    public double getMinimumDistanceAway () {
        return MINIMUM_DISTANCE_AWAY;
    }

    public void setX (double x) {
        point.set(0, x);
    }

    public void setY (double y) {
        point.set(1, y);
    }

    public void setAngle (double angle) {
        if (Math.abs(angle) <= 180)
            point.set(2, angle);
        else if (angle > 0)
            point.set(2, angle - 360);
        else
            point.set(2, angle + 360);
    }

    // Only used in WayPoint.java
    public void addToPoint (Double d) {
        point.add(d);
    }

    /**
     * Gets the current distance away from x, y coordinates.
     * @param targetX Target x coordinate.
     * @param targetY Target y coordinate.
     * @return Distance from point in inches.
     */

    public double getDistanceAway (double targetX, double targetY) {
        double deltaX = getX() - targetX;
        double deltaY = getY() - targetY;
        return Math.hypot(deltaX, deltaY);
    }

    public String toString () {
        return point.get(0) + ", " + point.get(1) + ", " + point.get(2);
    }
}
