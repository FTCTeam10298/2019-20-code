package org.firstinspires.ftc.teamcode

import java.util.ArrayList
import kotlin.math.abs
import kotlin.math.hypot

abstract class KPoint {

    // Index 0: x coordinate
    // Index 1: y coordinate
    // Index 2: angle (degrees)
    var point: ArrayList<Double> = ArrayList<Double>()
    open val MINIMUM_DISTANCE_AWAY: Double= 1.0

    fun Point() {
        this (0, 0, 0)
    }

    fun Point(angleDegrees: Double) {
        this (0, 0, angleDegrees)
    }

    fun Point(xCoordinate: Double, yCoordinate: Double) {
        this (xCoordinate, yCoordinate, 0)
    }

    fun Point(xCoordinate: Double, yCoordinate: Double, angleDegrees: Double) {
        point.add(xCoordinate)
        point.add(yCoordinate)

        when {
            abs(angleDegrees) <= 180 -> point.add(angleDegrees)
            angleDegrees > 0 -> point.add(angleDegrees - 360)
            else -> point.add(angleDegrees + 360)
        }
    }

    fun getPoint(): ArrayList<Double> {
        return point
    }

    fun getX(): Double {
        return point[0]
    }

    fun getY(): Double {
        return point[1]
    }

    fun getAngle(): Double {
        return point[2]
    }

    fun getMinimumDistanceAway(): Double {
        return MINIMUM_DISTANCE_AWAY
    }

    fun setX (x: Double) {
        point[0] = x
    }

    fun setY(y: Double) {
        point[1] = y
    }

    fun setAngle(angle: Double) {
        when {
            abs(angle) <= 180 -> point[2] = angle
            angle > 0 -> point[2] = angle - 360
            else -> point[2] = angle + 360
        }
    }

    // Only used in WayPoint.java
    fun addToPoint(d: Double) {
        point.add(d)
    }

    /**
     * Gets the current distance away from x, y coordinates.
     * @param targetX Target x coordinate.
     * @param targetY Target y coordinate.
     * @return Distance from point in inches.
     */

    fun getDistanceAway(targetX: Double, targetY: Double): Double {
        var deltaX: Double= getX() - targetX
        var deltaY: Double= getY() - targetY
        return hypot(deltaX, deltaY)
    }

    open override fun toString(): Double {
        return (point[0] + ", " + point[1] + ", " + point[2])
    }
}