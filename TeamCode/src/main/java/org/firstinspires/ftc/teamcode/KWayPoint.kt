package org.firstinspires.ftc.teamcode

class KWayPoint: Point() {

    fun WayPoint() {
        this(0, 0, 0, 0)
    }

    fun WayPoint(angleDegre: Double) {
        this(0, 0, angleDegre, 0)
    }

    // Index 3: index
    fun WayPoint(xCoordinate: Double, yCoordinate: Double, angleDegree: Double, index: Double) {
        super.(xCoordinate, yCoordinate, angleDegree)
        addToPoint(index)
    }

    fun getIndex(): Double {
        return super.getPoint().get(3)
    }

    override fun toString(): String {
        return super.toString() + ", " + getIndex()
    }
}