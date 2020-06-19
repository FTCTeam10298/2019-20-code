package org.firstinspires.ftc.teamcode

import kotlin.math.*

class KRoboPoint(xCoordinate: Double, yCoordinate: Double, angleDegrees: Double): KPoint(xCoordinate, yCoordinate, angleDegrees) {

    override val MINIMUM_DISTANCE_AWAY: Double= 1.0 //Minimum distance away from point to move
    private val MINIMUM_ANGLE_AWAY: Double= 1.0
    private val DISTANCE_BETWEEN_WHEELS: Double= 15.0
    private val COUNTS_PER_MOTOR_REV: Double= 28.0  //Rev HD Hex v2.1 Motor encoder
    private val GEARBOX_RATIO: Double= 20.0 //40 for 40:1, 20 for 20:1
    private val DRIVE_GEAR_REDUCTION: Double= 1.0   //This is > 1.0 if geared for torque
    private val WHEEL_DIAMETER_INCHES: Double= 3.875    //For figuring circumference
    private val ENC_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) // about 46
    private val COUNTS_PER_INCH: Double= 46.000912583980071241587049026378;

    constructor(): this(0.0, 0.0, 0.0)

    constructor(xCoordinate: Double, yCoordinate: Double): this(xCoordinate, yCoordinate, 0.0)


    fun getDistanceBetweenWheels(): Double{
        return DISTANCE_BETWEEN_WHEELS
    }

    /**
     * Gets the current distance away from a point.
     * @param p Point.
     * @return Distance from point in inches.
     */

    fun getDistanceAway(p: KPoint): Double {
        var deltaX: Double= getPoint()[0] - p.getX()
        var deltaY: Double= getPoint()[1] - p.getY()
        return hypot(deltaX, deltaY)
    }

    /**
     * Gets the current angle away from the point.
     * @param p Point from which we compare.
     * @return Angle from point in degrees.
     */
    fun getAngleAway(p: KPoint): Double{
        return when {
            abs(p.getAngle() - getAngle()) <= 180 -> p.getAngle() - getAngle()
            p.getAngle() - getAngle() > 0 -> p.getAngle() - getAngle() - 360
            else -> p.getAngle() - getAngle() + 360
        }
    }


    /**
     * Gets the heading error from a given point.
     * @param p Target point.
     * @return Heading error (degrees).
     */
    fun getHeadingError (p: KPoint): Double{
        return Math.toDegrees(atan2(getX() - p.getX(), getY() - p.getY()))
        - getAngle()
    }

    /**
     * Checks if the current position is close enough to a given point to begin the next action.
     * @param p Point.
     * @return True if the distance from the point is less than the minimum distance away from
     *         the point, false if otherwise.
     */
    fun isPointReached(p: KPoint): Boolean {
        return getDistanceAway(p) < MINIMUM_DISTANCE_AWAY
    }

    /**
     * Checks if the current angle is close enough to the point to begin the next action.
     * @param p Point from which we compare.
     * @return True if the angle away from the target is less than the minimum angle away from the
     *         point, false if otherwise.
     */
    fun isAngleReached(p: KPoint): Boolean{
        return Math.abs(getAngleAway(p)) < MINIMUM_ANGLE_AWAY
    }

    /**
     * Checks if the current coordinates and angle are close enough to the point to begin the next
     * action.
     * @param p Target point.
     * @return True if the distance from the point is less than the minimum distance away from the
     *         point and if the angle away from the target is less than the minimum angle away from
     *         the point, false if otherwise.
     */
    fun isPositionReached(p: KPoint): Boolean {
        return isPointReached(p) && isAngleReached(p)
    }


    fun updatePosition(changeInLeftEncoder: Double,changeInRightEncoder: Double, leftEncoder: Double, rightEncoder: Double) {
        var changeInLeftInches: Double= changeInLeftEncoder / COUNTS_PER_INCH
        var changeInRightInches: Double= changeInRightEncoder / COUNTS_PER_INCH
        var changeInAngle: Double= -(changeInLeftInches - changeInRightInches) / DISTANCE_BETWEEN_WHEELS
        var radius: Double= DISTANCE_BETWEEN_WHEELS/2 *
                (changeInLeftInches + changeInRightInches) /
                (changeInLeftInches - changeInRightInches)
        var changeInX: Double= radius * (1 - cos(changeInAngle))
        var changeInY: Double= radius * sin(changeInAngle)
        var x: Double= changeInX * cos(Math.toRadians(getAngle()))
        - changeInY * sin(Math.toRadians(getAngle()))
        var y: Double= changeInX * sin(Math.toRadians(getAngle()))
        - changeInY * cos(Math.toRadians(getAngle()))
        setX(getX() + x)
        setY(getY() + y)
        setAngle(-(leftEncoder - rightEncoder) / DISTANCE_BETWEEN_WHEELS)
        //setAngle(getAngle() + Math.toDegrees(changeInAngle))
        if (getAngle() > 180)
            setAngle(getAngle() - 360)
        if (getAngle() < -179)
            setAngle(getAngle() + 360)
    }

    override fun toString(): String {
        return super.toString()
    }
}