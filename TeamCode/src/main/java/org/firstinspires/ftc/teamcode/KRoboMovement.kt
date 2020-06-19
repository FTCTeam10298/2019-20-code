package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.Range
import hallib.HalDashboard
import java.lang.Math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class KRoboMovement: KOlivanieV3Hardware() {

    var dashboard: HalDashboard= HalDashboard.getInstance()

    enum class State {
        INIT,
        BUSY,
        DONE,
        TIMEOUT
    }
    private var prevErrorX: Double= 0.0
    private var prevErrorY: Double= 0.0
    private var prevErrorA: Double= 0.0
    private var sumErrorX: Double= 0.0
    private var sumErrorY: Double= 0.0
    private var sumErrorA: Double= 0.0
    private var sumMaxD: Double= 1.0
    private var sumMaxA: Double= 1.0
    var current: Coordinate= Coordinate(0.0, 0.0, 0.0)

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
    fun goToPosition(target: KCoordinate, maxPower: Double, distancePID: PID,
                     anglePID: PID, distanceMin: Double, angleDegMin: Double, iState: State): State {
        // Start by setting all speeds and error values to 0 and moving into the next state
        var state = iState
        when (state) {
            State.INIT -> {
                setSpeedZero()
                prevErrorX = 0.0
                prevErrorY = 0.0
                prevErrorA = 0.0
                sumErrorX = 0.0
                sumErrorY = 0.0
                sumErrorA = 0.0
                state = State.BUSY
            }
            State.BUSY -> {
                // Set the current position
                current.setCoordinate(globalRobot.getX(), globalRobot.getY(), Math.toDegrees(globalRobot.getAngle()))
                // Find the error in distance and angle, ensuring angle does not exceed 2*Math.PI
                var distanceError: Double= kotlin.math.hypot(current.x - target.getX(), current.y - target.getY())
                var angleError: Double= (target.getY() - current.angle) % (2*PI)
                if (angleError > PI)
                    angleError -= 2*PI
                // Find the absolute angle error
                var absAngleError: Double= atan2(target.getY() - current.y, target.getX() - current.x)
                - current.getAngle();
                // Convert the largest allowed error into radians to use in calculations
                var angleMin: Double= Math.toRadians(angleDegMin)
                // Check to see if we've reached the desired position already
                if (distanceError <= distanceMin && abs(angleError) <= angleMin) {
                    state = State.DONE
                }
                // Calculate the error in x and y and use the PID to find the error in angle
                var errx: Double= -sin(absAngleError) * distanceError
                var erry: Double= cos(absAngleError) * distanceError
                var dx: Double= errx * distancePID.propo
                var dy: Double= erry * distancePID.propo
                var da: Double= (angleError * anglePID.propo)
                dashboard = HalDashboard.getInstance()
                dashboard.displayPrintf(5, "Target Robot X, Error X: %f, %f", target.getX(), errx)
                dashboard.displayPrintf(6, "Target Robot Y, Error Y: %f, %f", target.getY(), erry)
                dashboard.displayPrintf(7, "Distance Error: %f", distanceError)
                dashboard.displayPrintf(8, "Current X,Y,A: %f, %f, %f", current.x,current.y,Math.toDegrees(current.angle))
                dashboard.displayPrintf(9, "angleError, target angle: %f, %f", Math.toDegrees(angleError), Math.toDegrees(target.getAngle()))
                dashboard.displayPrintf(10, "absAngleError: %f", Math.toDegrees(absAngleError))
                dashboard.displayPrintf(11, "Raw L, Raw C, Raw R: %d, %d, %d", rightCollector?.currentPosition, leftCollector?.currentPosition, tape?.currentPosition)

                // I and D terms are not being currently used
//                I plan to fix that. :|

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
                var dTotal: Double= abs(dx) + abs(dy) + 1E-6
                var newSpeedx: Double= Range.clip(dx, -3.0, 3.0)// / dTotal
                var newSpeedy: Double= Range.clip(dy, -3.0, 3.0)// / dTotal
                var newSpeedA: Double= Range.clip(da, -3.0, 3.0)

                dashboard.displayPrintf(12, "Speedx, SpeedY, SpeedA %f, %f, %f", newSpeedx, newSpeedy, newSpeedA)
                setSpeedAll(newSpeedx, newSpeedy, newSpeedA, .05, maxPower)
            }
            State.DONE -> {
                setSpeedZero()
            }
        }
        return state
    }

    fun DoGoToPosition(target: KCoordinate,
                       maxPower: Double,
                       distancePID: PID,
                       anglePID: PID,
                       distanceMin: Double,
                       angleDegMin: Double,
                       state: State,
                       opmodeisactive: LinearOpMode
    ): State {
        var current: State= state
        while (current != State.DONE && current != State.TIMEOUT && opmodeisactive.opModeIsActive()) {
            updatePosition()
            current = goToPosition(target, maxPower, distancePID, anglePID, distanceMin, angleDegMin, current)
        }
        setSpeedZero()
        updatePosition()
        return current
    }

    fun StraightGoToPosition(target: KCoordinate, maxPower: Double, distanceMin: Double, opmodeisactive: LinearOpMode) {
        DoGoToPosition(target, maxPower, PID(.1, 0.0, 0.0), PID(.5, 0.0, 0.0),
                distanceMin, 5.0, State.INIT, opmodeisactive)
    }

    fun TurnGoToPosition(target: KCoordinate, maxPower: Double, angleDegMin: Double, opmodeisactive: LinearOpMode) {
        DoGoToPosition(target, maxPower, PID(0.01, 0.0, 0.0),
                PID(.5, 0.0, 0.0), 8.0, angleDegMin, State.INIT, opmodeisactive)
    }

}