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

    private ArrayList<WayPoint> path = new ArrayList<WayPoint>();
    private RoboPoint roboPoint = new RoboPoint();

    private static int currectWayPoint = 0;

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
        roboPoint.setAngle(startAngle);
        roboPoint.setX(startX);
        roboPoint.setY(startY);
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
        return "Angle: " + roboPoint.getAngle() + ", x: " + roboPoint.getX() + ", y: "
                + roboPoint.getY();
    }
}