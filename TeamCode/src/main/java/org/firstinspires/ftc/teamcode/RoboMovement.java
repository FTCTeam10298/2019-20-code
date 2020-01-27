package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

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

    /**
     * Sets the motor powers to the correct power to go to the target position.
     * @param target The target Coordinate to drive to.
     * @param maxPower The maximum power allowed on the drive motors.
     * @param distancePID The PID for the x-y error.
     * @param anglePID The PID for the theta error.
     * @param distanceMin The minimum allowed distance away from the target to terminate.
     * @param angleDegMin The minimum allowed angle away from the target to terminate.
     * @param state The current State of the robot.
     * @return The new State of the robot.
     */
    public State goToPosition (Coordinate target, double maxPower, PID distancePID,
                               PID anglePID, double distanceMin, double angleDegMin, State state) {
        // Start by setting all speeds and error values to 0 and moving into the next state
        if (state == State.INIT) {
            setSpeedZero();
            prevErrorX = 0;
            prevErrorY = 0;
            prevErrorA = 0;
            sumErrorX = 0;
            sumErrorY = 0;
            sumErrorA = 0;
            state = State.BUSY;
        }
        else if (state == State.BUSY) {
            // Set the current position
            current.setCoordinate(globalRobot.getX(), globalRobot.getY(), Math.toDegrees(globalRobot.getAngle()));
            // Find the error in distance and angle, ensuring angle does not exceed 2*Math.PI
            double distanceError = Math.hypot(current.getX() - target.getX(), current.getY() - target.getY());
            double angleError = (target.getAngle() - current.getAngle()) % (2*Math.PI);
            if (angleError > Math.PI)
                angleError -= 2*Math.PI;
            // Find the absolute angle error
            double absAngleError = Math.atan2(target.getY() - current.getY(), target.getX() - current.getX())
                    - getWorldAngle_rad();
            // Convert the largest allowed error into radians to use in calculations
            double angleMin = Math.toRadians(angleDegMin);
            // Check to see if we've reached the desired position already
            if (distanceError <= distanceMin && Math.abs(angleError) <= angleMin) {
                state = State.DONE;
            }
            // Calculate the error in x and y and use the PID to find the error in angle
            double errx = -Math.sin(absAngleError) * distanceError;
            double erry = Math.cos(absAngleError) * distanceError;
            double dx = errx * distancePID.getPropo();
            double dy = erry * distancePID.getPropo();
            double da = (angleError * anglePID.getPropo());
            dashboard = HalDashboard.getInstance();
            dashboard.displayPrintf(5, "Target Robot X, Error X: %f, %f", target.getX(), errx);
            dashboard.displayPrintf(6, "Target Robot Y, Error Y: %f, %f", target.getY(), erry);
            dashboard.displayPrintf(7, "Distance Error: %f", distanceError);
            dashboard.displayPrintf(8, "Current X,Y,A: %f, %f, %f", current.getX(),current.getY(),Math.toDegrees(current.getAngle()));
            dashboard.displayPrintf(9, "angleError, target angle: %f, %f", Math.toDegrees(angleError), Math.toDegrees(target.getAngle()));
            dashboard.displayPrintf(10, "absAngleError: %f", Math.toDegrees(absAngleError));

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
            //
            double dTotal = Math.abs(dx) + Math.abs(dy) + 1E-6;
            double newSpeedx = Range.clip(dx, -3, 3);// / dTotal;
            double newSpeedy = Range.clip(dy, -3, 3);// / dTotal;
            double newSpeedA = Range.clip(da, -3, 3);

            dashboard.displayPrintf(11, "Speedx, SpeedY, SpeedA %f, %f, %f", newSpeedx, newSpeedy, newSpeedA);
            setSpeedAll(newSpeedx, newSpeedy, newSpeedA, .05, maxPower);
        }
        else if (state == State.DONE) {
            setSpeedZero();
        }
        return state;
    }

    public State DoGoToPosition (Coordinate target, double maxPower,
                                 PID distancePID, PID anglePID, double distanceMin,
                                 double angleDegMin, State state, LinearOpMode opmodeisactive) {
        State current = state;
        while (current != State.DONE && current != State.TIMEOUT && opmodeisactive.opModeIsActive()) {
            updatePosition();
            current = goToPosition(target, maxPower, distancePID, anglePID, distanceMin,
                    angleDegMin, current);
        }
        setSpeedZero();
        return current;
    }

    public void StraightGoToPosition (Coordinate target, double maxPower,
                                      double distanceMin, LinearOpMode opmodeisactive) {
        DoGoToPosition(target, maxPower, new PID(.1, 0, 0),
                new PID(.5, 0, 0), distanceMin, 5, State.INIT, opmodeisactive);
    }

    public void TurnGoToPosition (Coordinate target, double maxPower,
                                  double angleDegMin, LinearOpMode opmodeisactive) {
        DoGoToPosition(target, maxPower, new PID(0.01, 0 , 0),
                new PID(.5, 0, 0), 8, angleDegMin, State.INIT, opmodeisactive);
    }

}
