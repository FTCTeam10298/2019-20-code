package org.firstinspires.ftc.teamcode;

public class Global_Robot extends Coordinate{
    double ynot = 0.315; //.315
    double xnot = 14.756661709/2.0;

    public Global_Robot(double x, double y, double a) {
        super(x, y, a);
    }

    @Override
    public String toString() {
        return super.toString();
    }

    /**
     * Update the robot's global coordinates with inputs of the change in the encoders.
     * @param deltaL Change in the left encoder.
     * @param deltaC Change in the center encoder.
     * @param deltaR Change in the right encoder.
     */
    public void updatePosition(double deltaL, double deltaC, double deltaR) {
        double robotX = getX();
        double robotY = getY();
        double robotA = getAngle();
        robotA += (1/(2* xnot))*(deltaR - deltaL);
        double deltaY = (.5)*(deltaR + deltaL);
        double deltaX = (((ynot /(2* xnot))*(deltaL - deltaR)) + deltaC);
        robotY += deltaX * -Math.cos(robotA) + deltaY * Math.sin(robotA);
        robotX += deltaX * Math.sin(robotA) + deltaY * Math.cos(robotA);
        setX(robotX);
        setY(robotY);
        setAngle(robotA % (2 * Math.PI));
        if (getAngle() > Math.PI)
            setAngle(getAngle() - 2*Math.PI);
    }
}
