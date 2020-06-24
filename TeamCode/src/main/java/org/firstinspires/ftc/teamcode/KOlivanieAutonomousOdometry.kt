/* Copyright (c) 2016-19 Brain Stormz. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of Brain Stormz, nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.Range
import com.vuforia.Vuforia.init

import ftclib.FtcChoiceMenu
import ftclib.FtcMenu
import ftclib.FtcValueMenu
import hallib.HalDashboard
import java.lang.Runtime.getRuntime
import java.util.function.DoubleToLongFunction

@Autonomous(name="KOlivanie Autonamous Odometry", group ="Olivanie")
@Disabled
abstract class KOlivanieAutonomousOdometry: FtcMenu.MenuButtons, OpMode() {

    enum class RunMode {
        RUNMODE_AUTO,
        RUNMODE_DEBUG
    }

    enum class Alliance {
        BLUE,
        RED
    }

    enum class StartPosition {
        BUILDING1,
        BUILDING2,
        LOADING1,
        LOADING2
    }

    enum class Skystones {
        ZERO,
        ONE,
        TWO
    }

    enum class Stones {
        ZERO,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX
    }

    enum class Foundation {
        YES,
        NO,
    }

    enum class Park {
        WALL,
        BRIDGE,
        NONE
    }

    enum class Auto_path {
        PATH1,
        PATH2,
        PATH3,
        PATH4
    }

    // Menu option variables
    var runmode: RunMode = RunMode.RUNMODE_AUTO
    var delay = 0
    var alliance: Alliance = Alliance.RED
    var startposition: StartPosition = StartPosition.LOADING2
    var skystones: Skystones = Skystones.ONE
    var stones: Stones = Stones.ZERO
    var foundation: Foundation = Foundation.YES
    var park: Park = Park.WALL
    var path: Auto_path = Auto_path.PATH1

    /* Declare OpMode members. */
    private var dashboard: HalDashboard = HalDashboard.getInstance()
    var robot: KOlivanieV3Hardware = KOlivanieV3Hardware()
//    Robosition position = new Robosition()
    var roboPoint: KRoboPoint = KRoboPoint()

    val PROPORTIONAL_TERM: Double = 1.0
    val INTEGRAL_TERM: Double = 0.0
    val DERIVATIVE_TERM = 0.0

    val COUNTS_PER_MOTOR_REV= 28.0      // Rev HD Hex v2.1 Motor encoder
    val GEARBOX_RATIO= 20.0      // 40 for 40:1, 20 for 20:1
    val DRIVE_GEAR_REDUCTION= 24.0/15.0 // This is > 1.0 if geared for torque
    val WHEEL_DIAMETER_INCHES= 3.937007874015748 // For figuring circumference
    val DRIVETRAIN_ERROR= 1.04      // Error determined from testing
    val COUNTS_PER_INCH: Double = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR

    var errors= 0
    var state= 0
    var driveState= 0
    var count= 0
    var direction= 1
    var currentAngle: Double = 0.0
    var currentPositionL: Double = 0.0
    var currentPositionR: Double = 0.0
    var leftTarget: Double = 0.0
    var rightTarget: Double = 0.0
    var angleTarget: Double = 0.0
    var startingAngle: Double = 0.0
    var moveOn= false

    // code to run once after driver hits init
    override fun init() {
        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap)

        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry)

        // Run though the menu ---------------------------------------------------------------------
        //doMenus();

        if (alliance == Alliance.RED) {
            if (startposition == StartPosition.BUILDING1) {
                if (skystones == Skystones.ZERO) {
                    if (stones == Stones.ZERO) {
                        if (foundation == Foundation.YES) {
                            if (park == Park.WALL) {
                                path = Auto_path.PATH1
                            }
                            else if (park == Park.BRIDGE) {
                                path = Auto_path.PATH3
                            }
                        }
                    }
                }
            }
        } else if (alliance == Alliance.BLUE) {
            if (startposition == StartPosition.BUILDING1) {
                if (skystones == Skystones.ZERO) {
                    if (stones == Stones.ZERO) {
                        if (foundation == Foundation.YES) {
                            if (park == Park.WALL) {
                                path = Auto_path.PATH2
                            }
                            else if (park == Park.BRIDGE) {
                                path = Auto_path.PATH4
                            }
                        }
                    }
                }
            }
        }

        dashboard.displayPrintf(0, "Status: Ready to start")
        if (errors > 0)
            dashboard.displayPrintf(2, "!!! %d errors!", errors)

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    override fun init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    override fun start() {

        /**
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running")

        //driveToPoint(1, new WayPoint(12, 24, 0, 0));
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    override fun loop () {

        if (state == 0) {
            moveOn = DriveStraight(.5, 24.0)
            if (moveOn) {
                state ++
            }
        }

    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {
    }

    /**
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    /**
     * When in debug mode, gives the user the option to skip the current task and move onto the
     * next.
     * @param taskname Name of the task.
     * @param debug Checks runmode.
     * @param default_value Default of using this task
     * @return True if task is run, false otherwise.
     */
    fun DoTask(taskname: String, debug: RunMode, default_value: Boolean): Boolean{
        dashboard.displayPrintf(0, taskname);
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard.displayPrintf(1, "Press A to run, B to skip")
            if (gamepad1.a) {
                dashboard.displayPrintf(1, "Run")
                return true
            }
            if (gamepad1.b) {
                dashboard.displayPrintf(1, "Skip")
                //sleep
                return false
            }
        }
        return default_value
    }

    /**
     * Drives straight using PID
     * @deprecated Use driveToPoint instead, adds turning
     * @param power Power
     * @param inches Inches
     */
    fun PIDDriveStraight(power: Double, inches: Double) {

        var output: Double
        var desiredDistance: Double = inches
        var power: Double = power

        if (power > 0 && desiredDistance < 0) {
            power *= 1
        }
        if (power < 0 && desiredDistance > 0) {
            desiredDistance *= -1
        }

        var desiredX: Double = roboPoint.getX() + (desiredDistance * Math.sin(Math.toRadians(roboPoint.getAngle())))
        var desiredY: Double = roboPoint.getY() + (desiredDistance * Math.cos(Math.toRadians(roboPoint.getAngle())))
        var counter: Double = 0.0
        var previousTime: Double = getRuntime()
        var previousError: Double = roboPoint.getDistanceAway(desiredX, desiredY)
        var I: Double = 0.0

        var wayPoint: KWayPoint = KWayPoint(desiredX, desiredY, roboPoint.getAngle(), 0.0)

        while (!roboPoint.isPointReached(wayPoint)) {

            var leftEncoder: Double = (robot.leftDriveF?.getCurrentPosition()!!.toDouble() + robot.leftDriveB!!.getCurrentPosition()) / 2
            var rightEncoder: Double = (robot.rightDriveF?.getCurrentPosition()!!.toDouble() + robot.rightDriveB!!.getCurrentPosition()) / 2
            var currentTime: Double= getRuntime()
            var currentError: Double = roboPoint.getDistanceAway(desiredX, desiredY)

            I += currentError * (currentTime - previousTime)


            // PID
            output = if (power > 0)
                Range.clip(PROPORTIONAL_TERM * currentError, -1.0, power)
            else
                Range.clip(PROPORTIONAL_TERM * currentError, power, 1.0)
            output += INTEGRAL_TERM * I
            output += DERIVATIVE_TERM * ((currentError - previousError) / (currentTime - previousTime))

            // Set Power
            if (Math.abs(power * counter) > Math.abs(output))
//                robot.setPowerAll(output)
            /*else {
                if (power > 0)
                    robot.setPowerAll(Range.clip(power * counter, -1.0, power))
                else
                    robot.setPowerAll(Range.clip(power * counter, power, 1.0))
            }*/

            // Update values
            counter += 0.01
            /*roboPoint.updatePosition(
                    ((robot!!.leftDriveF!!.getCurrentPosition()
                            + robot!!.leftDriveB!!.getCurrentPosition())/2
                    ) - leftEncoder,
                    ((robot!!.rightDriveF!!.getCurrentPosition()
                            + robot!!.rightDriveB!!.getCurrentPosition())/2)
                            - rightEncoder,
                    robot.getLeftWheelEncoder(),
                    robot.getRightWheelEncoder()
            )*/
            previousError = currentError
            previousTime = currentTime
        }
    }

    fun PIDTurn (power: Double, degrees: Double) {

        var speed: Double = Math.abs(power)
        var output: Double
        var desiredAngle: Double = degrees + roboPoint.getAngle()

        if (desiredAngle > 180)
            desiredAngle -= 360
        else if (desiredAngle < -180)
            desiredAngle += 360

        var wayPoint: KWayPoint = KWayPoint(roboPoint.getX(), roboPoint.getY(), desiredAngle,0.0)

        var counter: Double = 0.0
        var previousTime: Double = getRuntime()
        var previousError: Double = roboPoint.getAngleAway(wayPoint)
        var I: Double = 0.0
        var right: Boolean = roboPoint.getAngleAway(wayPoint) > 0

        while (!roboPoint.isAngleReached(wayPoint)) {
//            var leftEncoder: Double = robot.getLeftWheelEncoder()
//            var rightEncoder: Double = robot.getRightWheelEncoder()
            var currentTime: Double = getRuntime()
            var currentError: Double = roboPoint.getAngleAway(wayPoint)

            I += currentError * (currentTime - previousTime)

            // PID
            output = Range.clip(PROPORTIONAL_TERM * currentError, -1.0, Math.abs(speed))
            output += INTEGRAL_TERM * I
            output += DERIVATIVE_TERM * ((currentError - previousError) / (currentTime - previousTime))

            // Set Power
            if (Math.abs(speed * counter) > Math.abs(output)) {
                if (right) {
//                    robot.setPowerLeft(Range.clip(output, 1.0, speed))
//                    robot.setPowerRight(-Range.clip(output, 1.0, speed))
                }
                else {
//                    robot.setPowerLeft(-Range.clip(output, 1.0, speed))
//                    robot.setPowerRight(Range.clip(output, 1.0, speed))
                }
            }
            else {
                if (right) {
//                    robot.setPowerLeft(Range.clip(Math.abs(speed * counter), 0.0, speed))
//                    robot.setPowerRight(-Range.clip(Math.abs(speed * counter), -speed,0.0))
                }
                else {
//                    robot.setPowerLeft(-Range.clip(Math.abs(speed * counter), -speed, 0.0))
//                    robot.setPowerRight(Range.clip(Math.abs(speed * counter), 0.0, speed))
                }
            }

            counter += 0.01
          /*  roboPoint.updatePosition(
                    robot.getLeftWheelEncoder() - leftEncoder,
                    robot.getRightWheelEncoder() - rightEncoder,
                    robot.getLeftWheelEncoder(),
                    robot.getRightWheelEncoder()
            )*/
            previousError = currentError
            previousTime = currentTime
        }
    }

    fun driveToPoint(power: Double, point: KPoint) {

        var speed: Double = Math.abs(power)
        var drive: Double
        var outputPIDStraight: Double
        var outputPIDTurn: Double
        var turn: Double
        var counter: Double = 0.0
        var previousTime: Double = getRuntime()
        var previousError: Double = roboPoint.getDistanceAway(point)
        var previousHeadingError: Double = roboPoint.getHeadingError(point)
        var I: Double = 0.0

        while (!roboPoint.isPointReached(point)) {

            var currentTime: Double = getRuntime()
            var currentError: Double = roboPoint.getDistanceAway(point)
            var currentHeadingError: Double = roboPoint.getHeadingError(point)
//            var leftEncoder: Double = robot.getLeftWheelEncoder()
//            var rightEncoder: Double = robot.getRightWheelEncoder()

            var wayPoint: KWayPoint = KWayPoint(currentHeadingError)

            var driveDirection = if (Math.abs(roboPoint.getAngleAway(wayPoint)) > 90)
                -1
            else
                1

            var turnDirection = if (roboPoint.getAngleAway(wayPoint) > 0)
                1
            else
                -1

            I += currentError * (currentTime - previousTime)

            // PID
            outputPIDStraight = Range.clip(PROPORTIONAL_TERM * currentError, -1.0, speed)
            outputPIDStraight += INTEGRAL_TERM * I
            outputPIDStraight += DERIVATIVE_TERM * ((currentError - previousError) / (currentTime - previousTime))
            outputPIDTurn = Range.clip(PROPORTIONAL_TERM * currentHeadingError, -1.0, speed)
            outputPIDTurn += INTEGRAL_TERM * I
            outputPIDTurn += DERIVATIVE_TERM * ((currentHeadingError - previousHeadingError) / (currentTime - previousTime))

            // Set Power
            if (Math.abs(speed * counter) > Math.abs(outputPIDStraight)) {
                drive = outputPIDStraight
                turn = outputPIDTurn
            }
            else {
                drive = Range.clip(speed * counter, -1.0, speed)
                turn = Range.clip(speed * counter, -1.0, speed)
            }

            drive *= driveDirection
            turn *= turnDirection

//            robot.setPowerRight(Range.clip(drive + turn, -1.0, 1.0))
//            robot.setPowerLeft(Range.clip(drive - turn, -1.0, 1.0))

            // Update values
            counter += 0.01;
            /*roboPoint.updatePosition(
                    robot.getLeftWheelEncoder() - leftEncoder,
                    robot.getRightWheelEncoder() - rightEncoder,
                    robot.getLeftWheelEncoder(),
                    robot.getRightWheelEncoder()
            )*/
            previousError = currentError
            previousTime = currentTime
        }
        PIDTurn(power, point.getAngle() - roboPoint.getAngle()) //Eventually add to above w/ PID
    }

    fun DriveStraight (power: Double, inches: Double): Boolean {
        var speed: Double = Range.clip(Math.abs(power), -1.0, 1.0)
        var error: Double = 0.0
        var proportional: Double = 1.0
        var output: Double
        var done = false
//        currentPositionL = robot.getLeftWheelEncoder() / (COUNTS_PER_INCH)
//        currentPositionR = robot.getRightWheelEncoder() / (COUNTS_PER_INCH)
        when (driveState) {
            0 -> {
                leftTarget = currentPositionL + (inches * COUNTS_PER_INCH)
                rightTarget = currentPositionR + (inches * COUNTS_PER_INCH)
                if (leftTarget > currentPositionL) {
                    direction = -1
                }
                driveState ++
                dashboard.displayPrintf(5, "drive state 0")
            }
            1 -> {
                output = Range.clip(speed * direction.toDouble() *  count, -speed, speed)
//                robot.setPowerAll(output)
                count ++
                if (Math.abs(output) >= Math.abs(Range.clip(proportional * (rightTarget - currentPositionR), -speed, speed))) {
                    driveState ++
                    count = 0
                }
                dashboard.displayPrintf(5, "drive state 1")
                dashboard.displayPrintf(6, "output: %f", Math.abs(output))
                dashboard.displayPrintf(
                        7,
                        "prop: %f",
                        Math.abs(Range.clip(proportional * (rightTarget - currentPositionR), -speed, speed))
                )
            }
            2 -> {
                error = ((currentPositionR - rightTarget) + (currentPositionL - leftTarget)) / 2.0
                output = Range.clip(
                        proportional * (rightTarget - currentPositionR) * direction.toDouble(),
                        -speed,
                        speed
                )
//                robot.setPowerAll(output)
                if (error < 1.0) {
//                    robot.setPowerAll(0.0)
                    driveState = 0
                    done = true
                }
                dashboard.displayPrintf(5, "drive state 2")
                dashboard.displayPrintf(6, "error %f", error)
            }
        }
        return done
    }

    fun DriveTurn (power: Double, degrees: Double): Boolean {
        var speed: Double = Range.clip(Math.abs(power), -1.0, 1.0)
        var radians: Double = Math.toRadians(degrees)
//        currentPositionL = robot.getLeftWheelEncoder()
//        currentPositionR = robot.getRightWheelEncoder()
        currentAngle = -(currentPositionL - currentPositionR) / roboPoint.getDistanceBetweenWheels()
        var angleError: Double = radians + currentAngle
        var direction=  1
        var output: Double
        var proportional: Double = 1.0
        var done = false
        if (driveState == 0) {
            startingAngle = currentAngle;
            angleTarget = radians + currentAngle;
            if (angleError > 0) {
                direction = -1
            }
            driveState ++
        }
        else if (driveState == 1) {
            output = Range.clip(speed * count, -speed, speed)
//            robot.setPowerLeft(output * direction)
//            robot.setPowerRight(output * direction * -1.0)
            count ++
            if (Math.abs(output) > Math.abs(Range.clip(
                            proportional
                            * ((angleTarget - startingAngle)- (currentAngle - startingAngle)) / angleTarget,
                            -speed, speed))
            ) {
                count = 0
                driveState++
            }
        }
        else if (driveState == 2) {
            output = Range.clip(
                    proportional * ((angleTarget - startingAngle)- (currentAngle - startingAngle)) / angleTarget,
                    -speed,
                    speed
            )
//            robot.setPowerLeft(output * direction)
//            robot.setPowerRight(output * direction * -1.0)
            if (Math.abs(angleTarget - currentAngle) < 1) {
//                robot.setPowerAll(0.0)
                driveState = 0
                done = true
            }
        }
        return done
    }
    /**
     * DriveRobotPosition drives the robot the set number of inches at the given power level.
     * @param inches How far to drive, can be negative
     * @param power Power level to set motors to
     */
    fun DriveRobotPosition(power: Double, inches: Double, smart_accel: Boolean) {
        var state = 0 // 0 = NONE, 1 = ACCEL, 2 = DRIVE, 3 = DECEL
        var position: Double = inches*COUNTS_PER_INCH

//        robot.driveSetRunToPosition()

      /*  if (smart_accel && power > 0.25) {
            robot.setPowerAll(0.25) // Use abs() to make sure power is positive
            state = 1 // ACCEL
        }
        else {
            robot.setPowerAll(Math.abs(power)); // Use abs() to make sure power is positive
        }*/

        var flOrigTarget: Int = robot.leftDriveF!!.getTargetPosition()
        var frOrigTarget: Int = robot.rightDriveF!!.getTargetPosition()
        var blOrigTarget: Int= robot.leftDriveB!!.getTargetPosition()
        var brOrigTarget: Int = robot.rightDriveB!!.getTargetPosition()
//        robot.driveAddTargetPosition(position.toInt(), position.toInt(), position.toInt(), position.toInt())

        for (i in (0..5)) {    // Repeat check 5 times, sleeping 10ms between,
        // as isBusy can be a bit unreliable
        /*while (robot.driveAllAreBusy()) {
            var flDrive: Int = robot.leftDriveF.getCurrentPosition()
            var frDrive: Int = robot.rightDriveF.getCurrentPosition()
            var blDrive: Int = robot.leftDriveB.getCurrentPosition()
            var brDrive: Int = robot.rightDriveB.getCurrentPosition()
            dashboard.displayPrintf(3, "Front left encoder: %d", flDrive)
            dashboard.displayPrintf(4, "Front right encoder: %d", frDrive)
            dashboard.displayPrintf(5, "Back left encoder: %d", blDrive)
            dashboard.displayPrintf(6, "Back right encoder %d", brDrive)

            // State magic
            if (state == 1 &&
                    (Math.abs(flDrive-flOrigTarget) > 2*COUNTS_PER_INCH ||
                            Math.abs(frDrive-frOrigTarget) > 2*COUNTS_PER_INCH ||
                            Math.abs(blDrive-blOrigTarget) > 2*COUNTS_PER_INCH ||
                            Math.abs(brDrive-brOrigTarget) > 2*COUNTS_PER_INCH
                            )
            ) {
                // We have gone 2 inches, go to full power
                robot.setPowerAll(Math.abs(power)) // Use abs() to make sure power is positive
                state = 2
            }
            else if (state == 2 &&
                    (Math.abs(flDrive-flOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2) ||
                            Math.abs(frDrive-frOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2) ||
                            Math.abs(blDrive-blOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2) ||
                            Math.abs(brDrive-brOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2)
                            )
            ) {
                // Cut power by half to DECEL
                robot.setPowerAll(Math.abs(power)/2) // Use abs() to make sure power is positive
                state = 3 // We are DECELing now
            }
            dashboard.displayPrintf(7, "State: %d (0=NONE,1=ACCEL,2=DRIVING,3=DECEL", state)
        }*/
        //sleep(10)
    }

//        robot.setPowerAll(0.0)
        // Clear used section of dashboard
        dashboard.displayText(3, "")
        dashboard.displayText(4, "")
        dashboard.displayText(5, "")
        dashboard.displayText(6, "")
        dashboard.displayText(7, "")
    }





    // MENU ----------------------------------------------------------------------------------------
    override fun isMenuUpButton(): Boolean { return gamepad1.dpad_up }

    override fun isMenuDownButton(): Boolean { return gamepad1.dpad_down }

    override fun isMenuEnterButton(): Boolean { return gamepad1.dpad_right }

    override fun isMenuBackButton(): Boolean { return gamepad1.dpad_left }

    private fun doMenus() {
        var modeMenu: FtcChoiceMenu<RunMode> = FtcChoiceMenu("Run Mode", null, this)
        var delayMenu: FtcValueMenu = FtcValueMenu("Delay:", modeMenu, this, 0.0, 20000.0, 1000.0, 0.0, "%.0f msec")
        var allianceMenu: FtcChoiceMenu<Alliance> = FtcChoiceMenu("Alliance:", delayMenu, this)
        var startPositionMenu: FtcChoiceMenu<StartPosition> =  FtcChoiceMenu("Start Position:", allianceMenu, this)
        var skystonesMenu: FtcChoiceMenu<Skystones> = FtcChoiceMenu("Number of Skystones:", startPositionMenu, this)
        var stonesMenu: FtcChoiceMenu<Stones> = FtcChoiceMenu("Number of Stones:", skystonesMenu, this)
        var foundationMenu: FtcChoiceMenu<Foundation> = FtcChoiceMenu("Move the Foundation:", stonesMenu, this)
        var parkMenu: FtcChoiceMenu<Park> = FtcChoiceMenu("Park :", foundationMenu, this)

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, delayMenu)
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, delayMenu)

        delayMenu.setChildMenu(allianceMenu)

        allianceMenu.addChoice("Red", Alliance.RED, true, startPositionMenu)
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, startPositionMenu)

        startPositionMenu.addChoice("Loading Zone Far", StartPosition.LOADING1, false, skystonesMenu)
        startPositionMenu.addChoice("Loading Zone Near", StartPosition.LOADING2, true, skystonesMenu)
        startPositionMenu.addChoice("Building Zone Far", StartPosition.BUILDING1, false, skystonesMenu)
        startPositionMenu.addChoice("Building Zone Near", StartPosition.BUILDING2, false, skystonesMenu)

        skystonesMenu.addChoice("None", Skystones.ZERO, false, stonesMenu)
        skystonesMenu.addChoice("One", Skystones.ONE, true, stonesMenu)
        skystonesMenu.addChoice("Two", Skystones.TWO, false, stonesMenu)

        stonesMenu.addChoice("None", Stones.ZERO, true, foundationMenu)
        stonesMenu.addChoice("One", Stones.ONE, false, foundationMenu)
        stonesMenu.addChoice("Two", Stones.TWO, false, foundationMenu)
        stonesMenu.addChoice("Three", Stones.THREE, false, foundationMenu)
        stonesMenu.addChoice("Four", Stones.FOUR, false, foundationMenu)
        if (
                skystonesMenu.getCurrentChoiceObject() == Skystones.ONE
                || skystonesMenu.getCurrentChoiceObject() == Skystones.ZERO
        )
            stonesMenu.addChoice("Five", Stones.FIVE, false, foundationMenu)
        if (skystonesMenu.getCurrentChoiceObject() == Skystones.ZERO)
            stonesMenu.addChoice("Six", Stones.SIX, false, foundationMenu)

        foundationMenu.addChoice("Yes", Foundation.YES, true, parkMenu)
        foundationMenu.addChoice("1", Foundation.NO, true, parkMenu)

        parkMenu.addChoice("Wall", Park.WALL, true)
        parkMenu.addChoice("Bridge", Park.BRIDGE, false)
        parkMenu.addChoice("None", Park.NONE, false)


        //FtcMenu.walkMenuTree(modeMenu);
        runmode = modeMenu.getCurrentChoiceObject()
        delay = delayMenu.getCurrentValue().toInt()
        alliance = allianceMenu.getCurrentChoiceObject()
        startposition = startPositionMenu.getCurrentChoiceObject()
        skystones = skystonesMenu.getCurrentChoiceObject()
        stones = stonesMenu.getCurrentChoiceObject()
        foundation = foundationMenu.getCurrentChoiceObject()
        park = parkMenu.getCurrentChoiceObject()

        dashboard.displayPrintf(13, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString())
        dashboard.displayPrintf(14, "Delay = %d msec", delay)
        dashboard.displayPrintf(15, "Alliance: %s (%s)", allianceMenu.getCurrentChoiceText(), alliance.toString())
        dashboard.displayPrintf(16, "Start position: %s (%s)", startPositionMenu.getCurrentChoiceText(), startposition.toString())
        dashboard.displayPrintf(17, "Number of Skystones: %s (%s)", skystonesMenu.getCurrentChoiceText(), skystones.toString())
        dashboard.displayPrintf(18, "Number of Stones: %s (%s)", stonesMenu.getCurrentChoiceText(), stones.toString())
        dashboard.displayPrintf(19, "Move Foundation: %s (%s)", foundationMenu.getCurrentChoiceText(), foundation.toString())
        dashboard.displayPrintf(20, "Park: %s (%s)", parkMenu.getCurrentChoiceText(), park.toString())
    }
    // END MENU ------------------------------------------------------------------------------------
}