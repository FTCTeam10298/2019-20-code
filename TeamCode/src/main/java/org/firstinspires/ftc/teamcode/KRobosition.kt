package org.firstinspires.ftc.teamcode

import java.util.ArrayList

/**
 * This is a tank drive odometry class for the 2019-2020 Skystone season. It holds the robot's
 * current position as well as a path with Waypoints (points to travel to). It can update the
 * robot's current position and calculate if the robot has reached the next Waypoint (or at least
 * come within a designated error).
 *
 * Note that whenever an individual value is called in a waypoint, that value must be cast as a
 * Double. All entries in those waypoints are Doubles.
 */

class KRobosition(startAngle: Double, startX: Double, startY: Double) {

    var path: ArrayList<KWayPoint> = ArrayList<KWayPoint>()
    var roboPoint: KRoboPoint= KRoboPoint()

    /**
     *  Primary constructor for Robosition
     * Note that (0,0) is at the center of the field and that 0 degrees is facing the loading zone.
     * @param startAngle Starting angle
     * @param startX Starting x coordinate
     * @param startY Starting y coordiante
     */

    init {
        roboPoint.setAngle(startAngle)
        roboPoint.setX(startX)
        roboPoint.setY(startY)
    }

    /**
     * Constructor for Robosition w/ preset angle of 0
     * @param startX Starting x coordinate
     * @param startY Starting y coordinate
     */
    constructor(startX: Double, startY: Double): this(0.0, startX, startY)

    /**
     * Constructor for Robosition w/ preset start position of 0, 0
     * @param startAngle Starting angle
     */
    constructor(startAngle: Double): this(startAngle, 0.0, 0.0)

    /**
     * Constructor for Robosition w/ preset start position of 0, 0 and angle of 0
     */

    constructor(): this(0.0, 0.0, 0.0)

    private var correctWayPoint: Int= 0

    private val COUNTS_PER_MOTOR_REV: Double= 28.0    // Rev HD Hex v2.1 Motor encoder
    private val GEARBOX_RATIO: Double= 20.0      // 40 for 40:1, 20 for 20:1
    private val DRIVE_GEAR_REDUCTION: Double= 1.0 // This is > 1.0 if geared for torque
    private val WHEEL_DIAMETER_INCHES: Double= 3.875 // For figuring circumference
    private val DRIVETRAIN_ERROR: Double= 1.0     // Error determined from testing
    private val COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
    (WHEEL_DIAMETER_INCHES * Math.PI) * DRIVETRAIN_ERROR;


    override fun toString(): String {
        return "Angle: " + roboPoint.getAngle() + ", x: " + roboPoint.getX() + ", y: "+ roboPoint.getY()
    }
}