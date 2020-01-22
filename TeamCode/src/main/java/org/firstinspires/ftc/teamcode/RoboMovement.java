package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import hallib.HalDashboard;

public class RoboMovement extends Olivanie_v2_Hardware{

    HalDashboard dashboard = HalDashboard.getInstance();

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
    Coordinate current = new Coordinate(0, 0, 0);

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
            current.setCoordinate(globalRobot.getX(), globalRobot.getY(), Math.toDegrees(globalRobot.getAngle()));
            double distanceError = Math.hypot(current.getX() - target.getX(), current.getY() - target.getY());
            double angleError = (target.getAngle() - current.getAngle()) % (2*Math.PI);
            if(angleError > Math.PI)
                angleError -= 2*Math.PI;
            double absAngleError = Math.atan2(target.getY() - current.getY(), target.getX() - current.getX())
                    - getWorldAngle_rad();
            double angleMin = Math.toRadians(angleDegMin);
            if (distanceError <= distanceMin && Math.abs(angleError) <= angleMin) {
                state = State.DONE;
            }
            double errx = -Math.sin(absAngleError) * distanceError;
            double erry = Math.cos(absAngleError) * distanceError;
            double dx = errx * distancePID.getPropo();
            double dy = erry * distancePID.getPropo();
            double da = (angleError * anglePID.getPropo());
            dashboard = HalDashboard.getInstance();
            dashboard.displayPrintf(5, "Target Robot X: %f, %f", target.getX(), errx);
            dashboard.displayPrintf(6, "Target Robot Y: %f, %f", target.getY(), erry);
            dashboard.displayPrintf(7, "Distance Error: %f", distanceError);
            dashboard.displayPrintf(8, "Current X,Y,A: %f,%f,%f", current.getX(),current.getY(),Math.toDegrees(current.getAngle()));
            dashboard.displayPrintf(9, "angleError, target: %f, %f", Math.toDegrees(angleError), Math.toDegrees(target.getAngle()));
            dashboard.displayPrintf(10, "absAngleError: %f", Math.toDegrees(absAngleError));
//            System.out.println("\n" + Math.toDegrees(current.getAngle()));
//            System.out.println(errx);
//            System.out.println(erry);
//            System.out.println(target.getY());
//            System.out.println(current.getY());
//            System.out.println(getYPos());
//            System.out.println(target.getY() - current.getY());
//            System.out.println(target.getX() - current.getX());
//            System.out.println(Math.toDegrees(absAngleError));
//            System.out.println(Math.toDegrees(angleError) + "\n");

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

            dashboard.displayPrintf(11, "Speedx, SpeedY, SpeedA %f,%f,%f", newSpeedx, newSpeedy, newSpeedA);
            setSpeedAll(newSpeedx, newSpeedy, newSpeedA);
        }
        else if (state == State.DONE) {
            setSpeedAll(0, 0, 0);
        }
        return state;
    }

    public State DoGoToPosition (Coordinate target, double power, PID distancePID, PID anglePID, double distanceMin,
                                 double angleDegMin, State state, LinearOpMode opmodeisactive) {
        State current = state;
        while (current != State.DONE && current != State.TIMEOUT && opmodeisactive.opModeIsActive()) {
            updatePosition();
            current = goToPosition(target, power, distancePID, anglePID, distanceMin,
                    angleDegMin, current);
        }
        setSpeedAll(0, 0, 0);
        return current;
    }

    public void StraightGoToPosition (Coordinate target, double power, double distanceMin, LinearOpMode opmodeisactive) {
        DoGoToPosition(target, power, new PID(.2, 0, 0), new PID(.5, 0, 0), distanceMin,
                5, State.INIT, opmodeisactive);
    }

    public void TurnGoToPosition (Coordinate target, double power, double angleDegMin, LinearOpMode opmodeisactive) {
        DoGoToPosition(target, power, new PID(0.01, 0 , 0), new PID(5, 0, 0), 8, angleDegMin, State.INIT, opmodeisactive);
    }

}
