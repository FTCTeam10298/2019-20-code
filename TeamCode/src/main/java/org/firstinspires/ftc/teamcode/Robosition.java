package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

/**
 * This is a tank drive odometry class for the 2019-2020 Skystone season. It holds the robot's
 * current position as well as a path with Waypoints (points to travel to). It can update the
 * robot's current position and calculate if the robot has reached the next Waypoint (or at least
 * come within a designated error).
 *
 * Note that whenever an individual value is called in a waypoint, that value must be cast as a
 * Double. All entries in those waypoints are Doubles.
 */

public class Robosition {

    private double angle; // All angle measurements are in degrees
    private double x; // All distance measurements are in inches
    private double y;
    private ArrayList<ArrayList> path = new ArrayList<ArrayList>();

    private static int currectWayPoint = 0;

    private static final double MINIMUM_DISTANCE_AWAY = 1; // Minimum distance away from point to move
    private static final double MINIMUM_ANGLE_AWAY = 1;
    private static final double DISTANCE_BETWEEN_WHEELS = 15;
    private static final double COUNTS_PER_MOTOR_REV  = 28.0;      // Rev HD Hex v2.1 Motor encoder
    private static final double GEARBOX_RATIO         = 20;      // 40 for 40:1, 20 for 20:1
    private static final double DRIVE_GEAR_REDUCTION  = 1; // This is > 1.0 if geared for torque
    private static final double WHEEL_DIAMETER_INCHES = 3.875; // For figuring circumference
    private static final double DRIVETRAIN_ERROR      = 1;      // Error determined from testing
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) * DRIVETRAIN_ERROR;


    /**
     * Constructor for Robosition
     * Note that (0,0) is at the center of the field and that 0 degrees is facing the loading zone.
     * @param startAngle Starting angle
     * @param startX Starting x coordinate
     * @param startY Starting y coordiante
     */
    public Robosition (double startAngle, double startX, double startY) {
        angle = startAngle;
        x = startX;
        y = startY;
        path.add(new <Double>ArrayList(3));
        path.get(0).add(startX);
        path.get(0).add(startY);
        path.get(0).add(startAngle);
    }

    /**
     * Constructor for Robosition w/ preset angle of 0
     * @param startX Starting x coordinate
     * @param startY Starting y coordinate
     */
    public Robosition (double startX, double startY) {
        this(0, startX, startY);
    }

    /**
     * Constructor for Robosition w/ preset start position of 0, 0
     * @param startAngle Starting angle
     */
    public Robosition (double startAngle) {
        this(startAngle, 0, 0);
    }

    /**
     * Constructor for Robosition w/ preset start position of 0, 0 and angle of 0
     */
    public Robosition () {
        this(0, 0, 0);
    }

    public String toString () {
        return "Angle: " + angle + ", x: " + x + ", y: " + y;
    }

    public double getAngle() {
        return angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getWayPointAngle(int point) {
        return (Double) path.get(point - 1).get(2);
    }

    public double getWayPointAngle() {
        return (Double) path.get(currectWayPoint).get(2);
    }

    public double getWayPointX (int point) {
        return (Double) path.get(point - 1).get(0);
    }

    public double getWayPointX () {
        return (Double) path.get(currectWayPoint).get(0);
    }

    public double getWayPointY (int point) {
        return (Double) path.get(point - 1).get(1);
    }

    public double getWayPointY () {
        return (Double) path.get(currectWayPoint).get(1);
    }

    public int getCurrectWayPoint () {
        return currectWayPoint + 1;
    }

    public double getCountsPerInch () {
        return  COUNTS_PER_INCH;
    }

    /**
     * Gets the current distance away from a point x, y.
     * @param targetX Target x coordinate.
     * @param targetY Target y coordinate.
     * @return Distance from Waypoint in inches.
     */

    public double getDistanceAway (double targetX, double targetY) {
        double deltaX = x - targetX;
        double deltaY = y - targetY;
        return Math.hypot(deltaX, deltaY);
    }

    /**
     * Gets the current distance away from the current Waypoint.
     * @return Distance from Waypoint in inches.
     */

    public double getDistanceAway () {
        double deltaX = x - (Double) path.get(currectWayPoint).get(0);
        double deltaY = y - (Double) path.get(currectWayPoint).get(1);
        return Math.hypot(deltaX, deltaY);
    }

    /**
     * Gets the current angle away from the point.
     * @param targetAngle Angle from which we compare.
     * @return Angle from point in degrees.
     */
    public double getAngleAway(double targetAngle) {
        if (Math.abs(targetAngle - angle) <= 180)
            return targetAngle - angle;
        else if (targetAngle - angle > 0)
            return targetAngle - angle - 360;
        else
            return targetAngle - angle + 360;
    }

    /**
     * Gets the current angle away from the Waypoint.
     * @return Angle from Waypoint in degrees.
     */
    public double getAngleAway () {
        if (Math.abs((Double) path.get(currectWayPoint).get(2) - angle) <= 180)
            return (Double) path.get(currectWayPoint).get(2) - angle;
        else if ((Double) path.get(currectWayPoint).get(2) - angle > 0)
            return (Double) path.get(currectWayPoint).get(2) - angle - 360;
        else
            return (Double) path.get(currectWayPoint).get(2) - angle + 360;
    }

    /**
     * Gets the heading error from a given point.
     * @param targetX Target x coordinate.
     * @param targetY Target y coordinate.
     * @return Heading error.
     */
    public double getHeadingError (double targetX, double targetY) {
        return Math.toDegrees(Math.atan2(x - targetX, y - targetY))
                - angle;
    }

    /**
     * Checks if the current position is close enough to a given point to begin the next action.
     * @param targetX Target x coordinate.
     * @param targetY Target y coordinate.
     * @return True if the distance from the point is less than the minimum distance away from
     *         the point, false if otherwise.
     */
    public boolean isPointReached(double targetX, double targetY) {
        return getDistanceAway(targetX, targetY) < MINIMUM_DISTANCE_AWAY;
    }

    /**
     * Checks if the current position is close enough to the Waypoint to begin the next action.
     * @return True if the distance from the Waypoint is less than the minimum distance away from
     *         the Waypoint, false if otherwise.
     */
    public boolean isPointReached() {
        return getDistanceAway() < MINIMUM_DISTANCE_AWAY;
    }

    /**
     * Checks if the current angle is close enough to the point to begin the next action.
     * @param targetAngle Target angle.
     * @return True if the angle away from the target is less than the minimum angle away from the
     *         point, false if otherwise.
     */
    public boolean isAngleReached(double targetAngle) {
        return Math.abs(getAngleAway(targetAngle)) < MINIMUM_ANGLE_AWAY;
    }

    /**
     * Checks if the current angle is close enough to the Waypoint to begin the next action.
     * @return True if the angle away from the target is less than the minimum angle away from the
     *         Waypoint, false if otherwise.
     */
    public boolean isAngleReached() {
        return Math.abs(getAngleAway((Double) path.get(currectWayPoint).get(2)))
                < MINIMUM_ANGLE_AWAY;
    }

    /**
     * Checks if the current coordinates and angle are close enough to the point to begin the next
     * action.
     * @param targetX Target x coordinate.
     * @param targetY Target y coordinate.
     * @param targetAngle Target angle.
     * @return True if the distance from the point is less than the minimum distance away from the
     *         point and if the angle away from the target is less than the minimum angle away from
     *         the point, false if otherwise.
     */
    public boolean isPositionReached(double targetX, double targetY, double targetAngle) {
        return isPointReached(targetX, targetY) && isAngleReached(targetAngle);
    }

    /**
     * Checks if the current coordinates and angle are close enough to the Waypoint to begin the
     * next action.
     * @return True if the distance from the Waypoint is less than the minimum distance away from
     *         the Waypoint and if the angle away from the target is less than the minimum angle
     *         away from the Waypoint, false if otherwise.
     */
    public boolean isPositionReached() {
        return isPointReached((Double) path.get(currectWayPoint).get(0),
                (Double) path.get(currectWayPoint).get(1))
                && isAngleReached((Double) path.get(currectWayPoint).get(2));
    }

    /**
     * Sets next Waypoint and adds it to the end of the path.
     * For each Waypoint:
     * Index = 0 x coordinate
     * Index = 1 y coordinate
     * Index = 2 angle
     * @param targetX       New target x coordinate
     * @param targetY       New target y coordinate
     * @param targetAngle   New target angle
     */
    public void setNextWayPoint(double targetX, double targetY, double targetAngle) {
        path.add(new <Double>ArrayList(3));
        currectWayPoint++;
        path.get(currectWayPoint).add(targetX);
        path.get(currectWayPoint).add(targetY);
        path.get(currectWayPoint).add(targetAngle);
    }

    /**
     * Updates x, y, and angle values based off of changes in encoders. Must be run at the end of
     * every loop.
     * @param changeinLeftEncoder  Average change between two left motor encoders (will be replaced
     *                             with free encoders once we get some.
     * @param changeinRightEncoder Average change between two right motor encoders (will be replaced
     *                             with free encoders once we get some.
     * @return                     True if the new position is within range of the target position,
     *                             false otherwise
     */
    public boolean updatePosition(double changeinLeftEncoder, double changeinRightEncoder) {
        double changeinLeftInches = changeinLeftEncoder * COUNTS_PER_INCH;
        double changeInRightInches = changeinRightEncoder * COUNTS_PER_INCH;
        double changeinAngle = Math.toDegrees((changeinLeftInches - changeInRightInches)
                / DISTANCE_BETWEEN_WHEELS);
        double radius = DISTANCE_BETWEEN_WHEELS/2 * (changeinLeftInches + changeInRightInches)
                / (changeinLeftInches - changeInRightInches);
        double changeInX = radius * (1 - Math.cos(Math.toRadians(changeinAngle)));
        double changeInY = radius * Math.sin(Math.toRadians(changeinAngle));
        x += changeInX * Math.cos(Math.toRadians(angle))
                - changeInY * Math.sin(Math.toRadians(angle));
        y += changeInX * Math.sin(Math.toRadians(angle))
                - changeInY * Math.cos(Math.toRadians(angle));
        angle += changeinAngle;
        if (angle > 180)
            angle -= 360;
        if (angle < -179)
            angle += 360;
        return isPointReached();
    }

    /**
     * Moves the robot to the next WayPoint.
     * @param robot The robot to move.
     */
    public void goToWayPoint (Olivanie_Hardware robot) {
        while (!isPointReached()) {

        }
    }
}
