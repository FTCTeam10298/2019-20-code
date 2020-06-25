package org.firstinspires.ftc.teamcode;

public class WayPoint extends Point {

    public WayPoint () {
        this(0, 0, 0, 0);
    }

    public WayPoint (double angleDegre) {
        this(0, 0, angleDegre, 0);
    }

    // Index 3: index
    public WayPoint (double xCoordinate, double yCoordinate, double angleDegree, double index) {
        super(xCoordinate, yCoordinate, angleDegree);
        addToPoint(index);
    }

    public double getIndex () {
        return super.getPoint().get(3);
    }

    @Override
    public String toString() {
        return super.toString() + ", " + getIndex();
    }
}
