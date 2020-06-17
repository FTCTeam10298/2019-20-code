package org.firstinspires.ftc.teamcode

import kotlin.math.cos
import kotlin.math.sin

class KGlobalRobot(x: Double, y: Double, a: Double): KCoordinate(x, y, a) {
    var ynot: Double= 0.315
    var xnot: Double= 14.756661709/2.0


    /**
     * Update the robot's global coordinates with inputs of the change in the encoders.
     * @param deltaL Change in the left encoder.
     * @param deltaC Change in the center encoder.
     * @param deltaR Change in the right encoder.
     */

    fun updatePosition(deltaL: Double, deltaC: Double, deltaR: Double) {
        var robotX: Double= getX()
        var robotY: Double= getY()
        var robotA: Double= getAngle()
        robotA += (1/(2* xnot))*(deltaR - deltaL)
        var deltaY: Double= (.5)*(deltaR + deltaL)
        var deltaX: Double= (((ynot /(2* xnot))*(deltaL - deltaR)) + deltaC)
        robotY += deltaX * -cos(robotA) + deltaY * sin(robotA)
        robotX += deltaX * sin(robotA) + deltaY * cos(robotA)
        setX(robotX)
        setY(robotY)
        setAngle(robotA % (2 * Math.PI))
        if (getAngle() > Math.PI)
            setAngle(2 * Math.PI)
    }

}