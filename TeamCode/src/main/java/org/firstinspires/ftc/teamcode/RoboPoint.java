package org.firstinspires.ftc.teamcode;

public class RoboPoint extends Point {

    private static final double MINIMUM_DISTANCE_AWAY = 1; // Minimum distance away from point to move
    private static final double MINIMUM_ANGLE_AWAY = 1;
    private static final double DISTANCE_BETWEEN_WHEELS = 15;
    private static final double COUNTS_PER_MOTOR_REV  = 28.0;      // Rev HD Hex v2.1 Motor encoder
    private static final double GEARBOX_RATIO         = 20;      // 40 for 40:1, 20 for 20:1
    private static final double DRIVE_GEAR_REDUCTION  = 1; // This is > 1.0 if geared for torque
    private static final double WHEEL_DIAMETER_INCHES = 3.875; // For figuring circumference
    private static final double ENC_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI); // about 46
    private static final double COUNTS_PER_INCH = 46.000912583980071241587049026378;

    public RoboPoint () {
        this (0, 0, 0);
    }

    public RoboPoint (double xCoordinate, double yCoordinate) {
        this (xCoordinate, yCoordinate, 0);
    }

    public RoboPoint (double xCoordinate, double yCoordinate, double angleDegrees) {
        super(xCoordinate, yCoordinate, angleDegrees);
    }


    /**
     * Gets the current distance away from a point.
     * @param p Point.
     * @return Distance from point in inches.
     */

    public double getDistanceAway (Point p) {
        double deltaX = getPoint().get(0) - p.getX();
        double deltaY = getPoint().get(1) - p.getY();
        return Math.hypot(deltaX, deltaY);
    }

    /**
     * Gets the current angle away from the point.
     * @param p Point from which we compare.
     * @return Angle from point in degrees.
     */
    public double getAngleAway(Point p) {
        if (Math.abs(p.getAngle() - getAngle()) <= 180)
            return p.getAngle() - getAngle();
        else if (p.getAngle() - getAngle() > 0)
            return p.getAngle() - getAngle() - 360;
        else
            return p.getAngle() - getAngle() + 360;
    }


    /**
     * Gets the heading error from a given point.
     * @param p Target point.
     * @return Heading error (degrees).
     */
    public double getHeadingError (Point p) {
        return Math.toDegrees(Math.atan2(getX() - p.getX(), getY() - p.getY()))
                - getAngle();
    }

    /**
     * Checks if the current position is close enough to a given point to begin the next action.
     * @param p Point.
     * @return True if the distance from the point is less than the minimum distance away from
     *         the point, false if otherwise.
     */
    public boolean isPointReached(Point p) {
        return getDistanceAway(p) < MINIMUM_DISTANCE_AWAY;
    }

    /**
     * Checks if the current angle is close enough to the point to begin the next action.
     * @param p Point from which we compare.
     * @return True if the angle away from the target is less than the minimum angle away from the
     *         point, false if otherwise.
     */
    public boolean isAngleReached(Point p) {
        return Math.abs(getAngleAway(p)) < MINIMUM_ANGLE_AWAY;
    }

    /**
     * Checks if the current coordinates and angle are close enough to the point to begin the next
     * action.
     * @param p Target point.
     * @return True if the distance from the point is less than the minimum distance away from the
     *         point and if the angle away from the target is less than the minimum angle away from
     *         the point, false if otherwise.
     */
    public boolean isPositionReached(Point p) {
        return isPointReached(p) && isAngleReached(p);
    }


    public void updatePosition (double changeInLeftEncoder, double changeInRightEncoder,
                                double leftEncoder, double rightEncoder) {
        double changeInLeftInches = changeInLeftEncoder / COUNTS_PER_INCH;
        double changeInRightInches = changeInRightEncoder / COUNTS_PER_INCH;
        double changeInAngle = -(changeInLeftInches - changeInRightInches) / DISTANCE_BETWEEN_WHEELS;
        double radius = DISTANCE_BETWEEN_WHEELS/2 * (changeInLeftInches + changeInRightInches)
                / (changeInLeftInches - changeInRightInches);
        double changeInX = radius * (1 - Math.cos(changeInAngle));
        double changeInY = radius * Math.sin(changeInAngle);
        double x = changeInX * Math.cos(Math.toRadians(getAngle()))
                - changeInY * Math.sin(Math.toRadians(getAngle()));
        double y = changeInX * Math.sin(Math.toRadians(getAngle()))
                - changeInY * Math.cos(Math.toRadians(getAngle()));
        setX(getX() + x);
        setY(getY() + y);
        setAngle(-(leftEncoder - rightEncoder) / DISTANCE_BETWEEN_WHEELS);
        //setAngle(getAngle() + Math.toDegrees(changeInAngle));
        if (getAngle() > 180)
            setAngle(getAngle() - 360);
        if (getAngle() < -179)
            setAngle(getAngle() + 360);
    }

    @Override
    public String toString () {
        return super.toString();
    }
}
