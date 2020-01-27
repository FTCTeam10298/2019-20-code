package org.firstinspires.ftc.teamcode;

public class Global_Robot {
    private Coordinate coordinate;
    double R = 2.2834645669291338582677165354331; //58 mm wheel radius
    double ynot = 1; //4 mm
    double xnot = 14.87963389/2.0;

    public Global_Robot(double x, double y, double a) {
        coordinate = new Coordinate(x, y, a);
    }

    public Coordinate getCoordinate () {
        return coordinate;
    }

    public void setX (double x){
        coordinate.setX(x);
    }

    public void setY (double y) {
        coordinate.setY(y);
    }

    public void setAngle (double a) {
        coordinate.setAngle(a);
    }

    public double getX () {
        return coordinate.getX();
    }

    public double getY () {
        return coordinate.getY();
    }

    public double getAngle () {
        return coordinate.getAngle();
    }

    @Override
    public String toString() {
        return "X: " + coordinate.getX() + "\nY: " + coordinate.getY() + "\nAngle: " + coordinate.getAngle();
    }

    /**
     * Update the robot's global coordinates with inputs of the change in the encoders.
     * @param thetaL Change in the left encoder.
     * @param thetaC Change in the center encoder.
     * @param thetaR Change in the right encoder.
     */
    public void updatePosition(double thetaL, double thetaC, double thetaR) {
        double robotX = coordinate.getX();
        double robotY = coordinate.getY();
        double robotA = coordinate.getAngle();
        robotA += (R/(2* xnot))*(thetaR - thetaL);
        double deltaY = (R/2)*(thetaR + thetaL);
        double deltaX = ((R)*(((ynot /(2* xnot))*(thetaR - thetaL)) + thetaC));
        robotY += deltaX * -Math.cos(robotA) + deltaY * Math.sin(robotA);
        robotX += deltaX * Math.sin(robotA) + deltaY * Math.cos(robotA);
        coordinate.setX(robotX);
        coordinate.setY(robotY);
        coordinate.setAngle(robotA % (2 * Math.PI));
        if (coordinate.getAngle() > Math.PI)
            coordinate.setAngle(coordinate.getAngle() - 2*Math.PI);
    }
}
