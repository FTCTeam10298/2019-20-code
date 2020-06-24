package org.firstinspires.ftc.teamcode

import java.lang.Math.PI
import java.util.*
import kotlin.math.hypot

open class KCoordinate(xPos: Double, yPos: Double, anglePos: Double) {

    private var x: Double= 0.0
    private var y: Double= 0.0
    private var angle: Double= 0.0

    /**
     * Creates a Coordinate with given values. All alternate constructors assume 0 for unstated variables.
     * @param xPos X position
     * @param yPos Y position
     * @param anglePos Angle
     */
    init {
        x = xPos
        y = yPos
        angle = Math.toRadians(anglePos)
    }

    constructor(xPos: Double, yPos: Double):this(xPos, yPos, 0.0)

    constructor(anglePos: Double):this(0.0, 0.0, anglePos)

    constructor():this(0.0, 0.0, 0.0)

    /**
     * Returns the x of the Coordinate
     * @return X
     */
    fun getX(): Double {
        return x
    }

    /**
     * Returns the y of the Coordinate
     * @return Y
     */
    fun getY(): Double {
        return y
    }

    /**
     * Returns the angle of the Coordinate
     * @return Angle
     */
    fun getAngle(): Double {
        return angle
    }

    fun setX(x: Double) {
        this.x = x
    }

    fun setY(y: Double) {
        this.y = y
    }

    fun setAngle(angle: Double) {
        this.angle = angle
    }

    fun setCoordinate(x: Double, y: Double, angle: Double) {
        this.setX(x)
        this.setY(y)
        this.setAngle(Math.toRadians(angle))
    }

    fun wrapAngle(angle: Double): Double {
        return angle % (2*PI)
    }

    /**
     * Gives the absolute value of the distance between the given Coordinate and the current Coordinate.
     * @param coordinate Coordinate to compare
     * @return distance from current Coordinate
     */
    fun distance(coordinate: KCoordinate): Double {
        return hypot(coordinate.x - x, coordinate.y - y)
    }

    /**
     * Gives the absolute value of the distance between the X and Y values and the current Coordinate.
     * @param targetX X
     * @param targetY Y
     * @return distance from current Coordinate
     */
    fun distance(targetX: Double, targetY: Double): Double {
        return hypot(targetX - x, targetY - y)
    }

    /**
     * Gives the error of the angle from the given Coordinate and the current Coordinate.
     * @param coordinate Coordinate to compare
     * @return angle error from current Coordinate
     */
    fun theta(coordinate: KCoordinate): Double {
        return wrapAngle(coordinate.angle - angle)
    }

    /**
     * Gives the error of the angle from the given angle and the current Coordinate.
     * @param targetA angle to compare
     * @return angle error from current Coordinate
     */
    fun theta(targetA: Double): Double {
        return  wrapAngle(targetA - angle)
    }

    fun equals(obj: Objects?): Boolean {
        return super.equals(obj)
    }

    override fun toString(): String {
        return "X: $x\nY: $y\nAngle: $angle"
    }
}