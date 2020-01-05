package org.firstinspires.ftc.teamcode;
import hallib.HalDashboard;

public class RoboMovement extends Olivanie_v2_Hardware{


    enum State {
        INIT,
        BUSY,
        DONE,
        TIMEOUT
    }
    private double prevErrorX = 0;
    private double prevErrorY = 0;
    private double prevErrorA = 0;
    private double sumErrorX = 0;
    private double sumErrorY = 0;
    private double sumErrorA = 0;
    private double sumMaxD = 1;
    private double sumMaxA = 1;

    public State goToPosition (Coordinate target, double power, PID distancePID, PID anglePID, double distanceMin,
                               double angleDegMin, State state) {
        if (state == State.INIT) {
            setSpeedAll(0, 0, 0);
            prevErrorX = 0;
            prevErrorY = 0;
            prevErrorA = 0;
            sumErrorX = 0;
            sumErrorY = 0;
            sumErrorA = 0;
            state = State.BUSY;
        }
        else if (state == State.BUSY) {
            Coordinate current = new Coordinate(getXPos(), getYPos(), Math.toDegrees(getWorldAngle_rad()));
            double distanceError = current.distance(target);
            double angleError = current.theta(target);
            double absAngleError = Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()) - getWorldAngle_rad();
            double angleMin = Math.toRadians(angleDegMin);
            if (distanceError <= distanceMin && Math.abs(angleError) <= angleMin) {
                state = State.DONE;
            }
            double errx = -Math.sin(absAngleError) * distanceError;
            double erry = Math.cos(absAngleError) * distanceError;
            double dx = errx * distancePID.getPropo();
            double dy = erry * distancePID.getPropo();
            double da = (angleError * anglePID.getPropo());
            System.out.println("\n" + Math.toDegrees(current.getAngle()));
            System.out.println(errx);
            System.out.println(erry);
            System.out.println(target.getY());
            System.out.println(current.getY());
            System.out.println(getYPos());
            System.out.println(target.getY() - current.getY());
            System.out.println(target.getX() - current.getX());
            System.out.println(Math.toDegrees(absAngleError));
            System.out.println(Math.toDegrees(angleError) + "\n");

            // I and D terms are not being currently used

//            sumErrorX += robot.getElapsedTime() * errx;
//            if (sumErrorX > sumMaxD)
//                sumErrorX = sumMaxD;
//            if (sumErrorY > sumMaxD)
//                sumErrorY = sumMaxD;
//            if (sumErrorA > sumMaxA)
//                sumErrorA = sumMaxA;
//            dx += sumErrorX * distancePID.getInteg();
//            dy += sumErrorY * distancePID.getInteg();
//            da += sumErrorA * anglePID.getInteg();
//            dx += (errx - prevErrorX) * distancePID.getDeriv()/ robot.getElapsedTime();
//            dy += (erry - prevErrorY) * distancePID.getDeriv()/ robot.getElapsedTime();
//            da += (angleError - prevErrorA) * distancePID.getDeriv()/ robot.getElapsedTime();
            double dTotal = Math.abs(dx) + Math.abs(dy) + 1E-6;
            double newSpeedx = (dx) * power / dTotal;
            double newSpeedy = (dy) * power / dTotal;
            double newSpeedA = (da) * power;

//            System.out.println(newSpeedx);
//            System.out.println(newSpeedy);
//            System.out.println(newSpeedA);
            setSpeedAll(newSpeedx, newSpeedy, newSpeedA);
        }
        else if (state == State.DONE) {
            setSpeedAll(0, 0, 0);
        }
        return state;
    }

    public State DoGoToPosition (Coordinate target, double power, PID distancePID, PID anglePID, double distanceMin,
                                 double angleDegMin, State state) {
        State current = state;
        while (current != State.DONE && current != State.TIMEOUT) {
            updatePosition();
            current = goToPosition(target, power, distancePID, anglePID, distanceMin,
                    angleDegMin, current);
        }
        setSpeedAll(0, 0, 0);
        return current;
    }

    public void StraightGoToPosition (Coordinate target, double power, double distanceMin) {
        DoGoToPosition(target, power, new PID(.0001, 0, 0), new PID(.01, 0, 0), distanceMin,
                5, State.INIT);
    }

    public void TurnGoToPosition (Coordinate target, double power, double angleDegMin) {
        DoGoToPosition(target, power, new PID(.001, 0 , 0), new PID(9, 0, 0), 5, angleDegMin, State.INIT);
    }

    public void grab () {
        StraightGoToPosition(new Coordinate(getCoordinate().getX()
                + (3 * Math.cos(getCoordinate().getAngle())), getCoordinate().getY(),
                getCoordinate().getAngle()), .1, 1);
        grabStone();
        StraightGoToPosition(new Coordinate(getCoordinate().getX()
                - (3 * Math.cos(getCoordinate().getAngle())), getCoordinate().getY(),
                getCoordinate().getAngle()), .1, 1);
    }

    public void drop () {
        StraightGoToPosition(new Coordinate(getCoordinate().getX()
                + (3 * Math.cos(getCoordinate().getAngle())), getCoordinate().getY(),
                getCoordinate().getAngle()), .1, 1);
        dropStone();
        StraightGoToPosition(new Coordinate(getCoordinate().getX()
                - (3 * Math.cos(getCoordinate().getAngle())), getCoordinate().getY(),
                getCoordinate().getAngle()), .1, 1);
    }
}
