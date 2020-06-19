package org.firstinspires.ftc.teamcode

class KWayPoint(xCoordinate: Double, yCoordinate: Double, angleDegree: Double, index: Double): Point() {

    // Index 3: index
    init {
        addToPoint(index)
    }

    constructor(): this(0.0, 0.0, 0.0, 0.0)

    constructor(angleDegree: Double): this(0.0, 0.0, angleDegree, 0.0)


    fun getIndex(): Double {
        return super.getPoint().get(3)
    }

    override fun toString(): String {
        return super.toString() + ", " + getIndex()
    }
}