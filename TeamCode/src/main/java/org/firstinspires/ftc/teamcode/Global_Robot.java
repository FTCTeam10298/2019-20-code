package org.firstinspires.ftc.teamcode;

public class Global_Robot {
    private Coordinate coordinate;
    double R = 2.2834645669291338582677165354331; //58 mm wheel radius
    double xnot = 0.15748031496062992125984251968504; //4 mm
    double ynot = 13.8125/2.0;

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

    public void updatePosition(double thetaL, double thetaC, double thetaR) {
        double worldX = coordinate.getX();
        double worldY = coordinate.getY();
        double worldA = coordinate.getAngle();
        worldY += (R/2)*(thetaR + thetaL);
        worldX += (R)*(((xnot/(2*ynot))*(thetaR - thetaL)) + thetaC);
        worldA += (R/(2*ynot))*(thetaL - thetaR);
        coordinate.setX(worldX);
        coordinate.setY(worldY);
        coordinate.setAngle(worldA % (2 * Math.PI));
    }
}
