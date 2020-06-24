/* Copyright (c) 2016-20 Brain Stormz. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import ftclib.FtcChoiceMenu
import ftclib.FtcMenu
import ftclib.FtcValueMenu
import hallib.HalDashboard
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import java.util.ArrayList

@Autonomous(name="K Olivanie Autonomous Linear", group ="Olivanie")
class KOlivanieAutonomousLinear: FtcMenu.MenuButtons, LinearOpMode() {

    companion object {
        //0 means skystone, 1 means yellow stone
        //-1 for debug, but we can keep it like this because if it works, it should change to either 0
        // or 255

        private var valMid: Double= 0.0
        private var valLeft: Double= 0.0
        private var valRight: Double= 0.0

        private var offsetX: Float= 0f/8f//changing this moves the three rects and the three circles
        // left or right, range : (-2, 2) not inclusive
        private var offsetY: Float= 0f/8f//changing this moves the three rects and circles up or
        // down, range: (-4, 4) not inclusive

        private var midPos: Array<Float> = arrayOf(4f/8f+offsetX, 4f/8f+offsetY)//0 = col, 1 = row
        private var leftPos: Array<Float> = arrayOf(2f/8f+offsetX, 4f/8f+offsetY)
        private var rightPos: Array<Float> = arrayOf(6f/8f+offsetX, 4f/8f+offsetY)
        //moves all rectangles right or left by amount. units are in ratio to monitor

        private var rectHeight: Float= .6f/8f
        private var rectWidth: Float= 1.5f/8f
    }

//    -----------Creating the options menu--------------

//    Choosing runmode
    enum class RunMode { RUNMODE_AUTO, RUNMODE_DEBUG }

//    Determining alliance
    enum class Alliance {
        BLUE,
        RED
    }

//    Building Zone vs Loading Zone. 1 is further from the audience.
    enum class StartPosition {
        BUILDING1,
        BUILDING2,
        LOADING1,
        LOADING2
    }

//    Amount of Skystones to attempt to retrive
    enum class Skystones {
        ZERO,
        ONE,
        TWO
    }

//    How many stones to collect in total
    enum class Stones {
        ZERO,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX
    }

//    Whether to move foundation
    enum class Foundation {
        YES,
        NO,
        PARTNER
    }

//    Whether/where to park
    enum class Park {
        WALL,
        BRIDGE,
        NONE
    }

//    Which auto path
    enum class Auto_path {
        PATH1,
        PATH2,
        PATH3,
        PATH4
    }

//    The position of the Skystone
    enum class SkystonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Menu option variables
    var runmode: RunMode= RunMode.RUNMODE_AUTO
    var delay: Int= 0
    var alliance: Alliance= Alliance.RED
    var startposition: StartPosition= StartPosition.LOADING2
    var skystones: Skystones= Skystones.ONE
    var skystonePosition: SkystonePosition= SkystonePosition.CENTER
    var stones: Stones= Stones.ZERO
    var foundation: Foundation= Foundation.YES
    var park: Park= Park.WALL
    var path: Auto_path= Auto_path.PATH1

    // Order of Stones depending on the Skystone
    var stoneOrderLB = arrayOf(0, 1, 2, 3, 4, 5)
    var stoneOrderC = arrayOf(4, 1, 5, 3, 2, 0)
    var stoneOrderRB = arrayOf(3, 0, 5, 4, 2, 1)
    var stoneOrderN = arrayOf(5, 4, 3, 2, 1, 0)

    /* Declare OpMode members. */
    var dashboard: HalDashboard? = null
    var robot: KRoboMovement= KRoboMovement()
    var target: KCoordinate= KCoordinate()

    // Skystone variables
    private var runtime: ElapsedTime= ElapsedTime()


    var time_a: Double= 0.0
    var dt: Double= 0.0

    private var rows: Int= 640
    private var cols: Int= 480

    var webcam: OpenCvCamera? = null
    var COUNTS_PER_MOTOR_REV: Double= 28.0      // Rev HD Hex v2.1 Motor encoder
    var GEARBOX_RATIO: Double= 20.0      // 40 for 40:1, 20 for 20:1
    var DRIVE_GEAR_REDUCTION: Double= 1.0         // This is > 1.0 if geared for torque
    var WHEEL_DIAMETER_INCHES: Double= 3.937007874015748 // For figuring circumference
    var DRIVETRAIN_ERROR: Double= 1.1       // Error determined from testing
    var COUNTS_PER_INCH: Double=
            (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR
    var COUNTS_PER_DEGREE: Double= COUNTS_PER_INCH*0.184 // Found by testing
    var multipier: Int= 1

    override fun runOpMode() {

        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap)

        // Initialize dashboard --------------------------------------------------------------------

        dashboard= HalDashboard.createInstance(telemetry)

        // Run though the menu ---------------------------------------------------------------------
        doMenus()

        dashboard?.displayPrintf(1, "For a little bot, I pack a biiiiiig punch!")

        if (alliance == Alliance.RED) {
            multipier = -1
            if (startposition == StartPosition.BUILDING1) {
                if (skystones == Skystones.ZERO) {
                    if (stones == Stones.ZERO) {
                        if (foundation == Foundation.YES) {
                            if (park == Park.WALL) {
                                path = Auto_path.PATH1
                            } else if (park == Park.BRIDGE) {
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
                            } else if (park == Park.BRIDGE) {
                                path = Auto_path.PATH4
                            }
                        }
                    }
                }
            }
        }


        var angle: Double = 0.0
        var direction: Double = 0.0
        if (alliance == Alliance.RED && (startposition == StartPosition.LOADING1 ||
                        startposition == StartPosition.LOADING2)) {
            angle = 180.0
            direction = -1.0
        } else {
            angle = 0.0
            direction = 1.0
        }


        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
//        robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION)
        robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER)
//        robot.driveSetTargetPosition(0, 0, 0, 0)

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        var cameraMonitorViewId: Int = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java,"Webcam 1"), cameraMonitorViewId)

        webcam?.openCameraDevice()//open camera
        webcam?.setPipeline(StageSwitchingPipeline())    //different stages
        webcam?.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT) //display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        // Sample while waiting for start
        while (!opModeIsActive() && !isStopRequested()) {
            dashboard?.displayPrintf(3, "Left Value: (%f)", valLeft)
            dashboard?.displayPrintf(4, "Middle Value: (%f)", valMid)
            dashboard?.displayPrintf(5, "Right Value: (%f)", valRight)
            dashboard?.displayPrintf(6, "Height: (\$d)", rows)
            dashboard?.displayPrintf(7, "Width: (%d)", cols)
            skystonePosition = if (valLeft < valMid && valLeft < valRight) {
                dashboard?.displayPrintf(9, "Skystone is LEFT")
                SkystonePosition.LEFT
            } else if (valMid < valLeft && valMid < valRight) {
                dashboard?.displayPrintf(9, "Skystone is CENTER")
                SkystonePosition.CENTER
            } else {
                dashboard?.displayPrintf(9, "Skystone is RIGHT")
                SkystonePosition.RIGHT
            }

            telemetry.update()
            sleep(100)
        }
        webcam?.stopStreaming()
        webcam?.closeCameraDevice()

        // Set starting location based off of menu options
        if (alliance == Alliance.BLUE) {
            when (startposition) {
                StartPosition.LOADING2 -> {
                    robot.setGlobalRobot(-63.0, 36.0, 0.0)
                }
                StartPosition.LOADING1 -> {
                    robot.setGlobalRobot(-63.0, 60.0, 0.0)
                }
                StartPosition.BUILDING2 -> {
                    robot.setGlobalRobot(-63.0, 84.0, Math.PI)
                }
                else -> robot.setGlobalRobot(-63.0, 108.0, Math.PI)
            }
        }else{
            when (startposition) {
                StartPosition.LOADING2 -> {
                    robot.setGlobalRobot(63.0, 36.0, Math.PI)
                }
                StartPosition.LOADING1 -> {
                    robot.setGlobalRobot(63.0, 60.0, Math.PI)
                }
                StartPosition.BUILDING2 -> {
                    robot.setGlobalRobot(63.0, 84.0, 0.0)
                }
                else -> robot.setGlobalRobot(63.0, 108.0, 0.0)
            }
        }

        // Determine the order of stones depending on the Skystone location
        var stoneOrder = stoneOrderLB
        if (skystonePosition == SkystonePosition.CENTER) {
            stoneOrder = stoneOrderC
        }else if ((alliance == Alliance.BLUE && skystonePosition == SkystonePosition.RIGHT) ||
                (alliance == Alliance.RED && skystonePosition == SkystonePosition.LEFT)) {
            stoneOrder = stoneOrderRB
        }

        if (skystones == Skystones.ZERO)
            stoneOrder = stoneOrderN

        // Find the total amount of stones being cycled
        var totalStones: Int= 0
        totalStones = when (stones) {
            Stones.ZERO -> 0
            Stones.ONE -> 1
            Stones.TWO -> 2
            Stones.THREE -> 3
            Stones.FOUR -> 4
            Stones.FIVE -> 5
            else -> 6
        }
        if (skystones == Skystones.ONE)
            totalStones =+ 1
        else if (skystones == Skystones.TWO)
            totalStones =+ 2

        /**
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard?.displayPrintf(0, "Status: Running")

//        // For testing odometry code
//        if (DoTask("Odometry Test", runmode, false)) {
//            dashboard.displayPrintf(2, "%f", robot.getX())
//            dashboard.displayPrintf(3, "%f", robot.getY())
//            dashboard.displayPrintf(4, "%f",
//                    Math.toDegrees(robot.getWorldAngle_rad()) + 180);
////            RoboMovement.State current = RoboMovement.State.INIT;
////            double y = robot.getY() + 72;
////            robot.DoGoToPosition(new Coordinate(robot.getX(), y,
////                            Math.toDegrees(robot.getWorldAngle_rad())), 0.2,
//////                    new PID(.001, 0 , 0), new PID(9, 0, 0), 1,
//////                    3, current);
////            while (current != RoboMovement.State.DONE && current != RoboMovement.State.TIMEOUT) {
////                dashboard.displayPrintf(2, "Global X: %f", robot.getX());
////                dashboard.displayPrintf(3, "Global Y: %f", robot.getY());
////                dashboard.displayPrintf(4, "Global Angle: %f", Math.toDegrees(
////                robot.getWorldAngle_rad()) + 180);
////                robot.updatePosition();
////                current = robot.goToPosition(new Coordinate(robot.getX(), y,
////                            Math.toDegrees(robot.getWorldAngle_rad())), 0.2,
////                    new PID(.001, 0 , 0), new PID(9, 0, 0), 1,
////                    3, current);
////                dt = getRuntime() - time_a;
////                time_a = getRuntime();
////                dashboard.displayPrintf(11, "Loop Time: %f", dt);
////            }
//
//
////            for (int i = 0; i < stoneOrderLB.length - 4; i++){
////                target.setCoordinate(-40 - i*2, stoneOrder[i] * 8 + 13, 0);
////                robot.StraightGoToPosition(target, 1.3, 1);
////                grab();
////                target.setCoordinate(-42, 140, 0);
////                robot.StraightGoToPosition(target, 1.3, 11);
////                drop();
////            }
////            robot.setSpeedAll(0, 0, 0);
////            target.setCoordinate(-42, 72, 0);
////            robot.StraightGoToPosition(target, 1.3, 1);
////            robot.setSpeedAll(0, 0, 0);
//        }
//
//        // For testing drive train motors and encoders
//        if (DoTask("Raw motor test", runmode, false)) {
//            robot.driveSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.setPowerAll(.5);
//            sleep(3000);
//            robot.setPowerAll(0);
//            sleep(1000);
//            robot.setPowerAll(-.5);
//            sleep(3000);
//            robot.setPowerAll(0);
//        }
//
//        if (DoTask("Chassis motor test", runmode, false)) {
//            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.setPowerAll(0.5);
//
//            robot.rightDriveB.setTargetPosition(5000);
//            while (robot.rightDriveB.isBusy()) {
//                sleep(10);
//            }
//
//            robot.leftDriveB.setTargetPosition(5000);
//            while (robot.leftDriveB.isBusy()) {
//                sleep(10);
//            }
//
//            robot.rightDriveF.setTargetPosition(5000);
//            while (robot.rightDriveF.isBusy()) {
//                sleep(10);
//            }
//
//            robot.leftDriveF.setTargetPosition(5000);
//            while (robot.leftDriveF.isBusy()) {
//                sleep(10);
//            }
//        }
//        if (DoTask("Drive test - backwards", runmode, false))
//            DriveRobotPosition(.5, -10, false);
//
//        if (DoTask("Spin test - clockwise", runmode, false))
//            DriveRobotTurn(.4, 360 * 5);
//
//        if (DoTask("Spin test - counter-clockwise", runmode, false))
//            DriveRobotTurn(.4, -360 * 5);
//
//        if (DoTask("Drive test (72 inches)", runmode, false))
//            DriveRobotPosition(.4, 72, false);
//
//        if (DoTask("Drive test (-72 inches)", runmode, false))
//            DriveRobotPosition(.4, -72, false);
//
//        if (DoTask("Strafe test", runmode, false))
//            DriveSideways(.4, 72);
//
//        if (DoTask("Strafe test -", runmode, false))
//            DriveSideways(.4, -72);
//
//        if (DoTask("Diagonal test 1", runmode, false))
//            DriveDiagonal(.4, 36, true);
//
//        if (DoTask("Diagonal test 2", runmode, false))
//            DriveDiagonal(.4, 36, false);

        // Pause the program for the selected delay period
        sleep(delay.toLong())
        if (DoTask("Test", runmode, false)) {
            robot.setGlobalRobot(0.0,0.0,0.0)
            for (i in (0..2)) {
                target.setCoordinate(target.getX(), target.getY() + 72, target.getAngle())
                robot.StraightGoToPosition(target, .2, .1, this)
                sleep(5000)
                target.setCoordinate(target.getX(), target.getY() - 72, target.getAngle())
                robot.StraightGoToPosition(target, .2, .1, this)
                sleep(5000)

//                target.setCoordinate(0, 0, 135);
//                robot.DoGoToPosition(target, .2, new PID(0, 0, 0),
//                        new PID (.5, 0, 0), 10, 1, RoboMovement.State.INIT, this);
//                sleep(5000);
//                target.setCoordinate(0, 0, 0);
//                robot.DoGoToPosition(target, .2, new PID(0, 0, 0),
//                        new PID (.5, 0, 0), 10, 1, RoboMovement.State.INIT, this);
//                sleep(5000);
            }
        }

        // Init
        if (DoTask("Init", runmode, true)) {
        }

        if (DoTask("Drive", runmode, true)) {
            // --------------------------These are all paths without stones-------------------------
            if (totalStones == 0) {
                // ----------------These are all paths that only move the foundation----------------
                if (foundation == Foundation.YES) {
                    target.setCoordinate(-28 * direction, 126.0, angle + 180)
                    robot.StraightGoToPosition(target, 1.0, 1.0,
                            this)
                    robot.closeFoundation()
                    sleep(500)
                    target.setCoordinate(-60 * direction, 126.0, angle + 180)
                    robot.StraightGoToPosition(target, 1.0, 1.0,
                            this)
                    robot.openFoundation()
                    sleep(500)
                    target.setCoordinate(-60 * direction, 84.0, angle + 180)
                    robot.StraightGoToPosition(target, 1.0, 2.0,
                            this)
                    if (park == Park.WALL) // We Park by Wall
                        target.setCoordinate(-60 * direction, 72.0, angle + 180)
                    else if (park == Park.BRIDGE) //We Park by Bridge
                        target.setCoordinate(-36 * direction, 72.0, angle + 180)
                    robot.StraightGoToPosition(target, 1.0, 1.0,
                            this)
                    robot.setSpeedZero()

                // -------------------------These are paths that only park--------------------------
                }else{
                    // Parking against the wall
                    if (park == Park.WALL) {
                        target.setCoordinate(-63 * direction, 72.0, angle)
                        robot.StraightGoToPosition(target, 1.0, 1.0, this)

                    // Parking against the bridge
                    }else if (park == Park.BRIDGE) { // We Park by the Bridge
                        target.setCoordinate(-36 * direction, 72.0, angle)
                        robot.StraightGoToPosition(target, 1.0, 1.0, this)
                    }
                    robot.setSpeedZero()
                }

            // ----------------------------These are all paths with stones--------------------------
            }else{

                // ------------These are all paths where our partner moves the Foundation-----------
                if (foundation == Foundation.PARTNER) {
                    for (i in (0..totalStones)) {
                        if (i == 0) {
                            target.setCoordinate(-40 * direction, stoneOrder[i] * 8 + 13.toDouble(), -90.0)
                        }else {
                            target.setCoordinate(-40 * direction, stoneOrder[i] * 8 + 13.toDouble(), -90.0)
                            robot.StraightGoToPosition(target, .5, 1.0, this)
                            grab(direction)
                            target.setCoordinate(target.getX(), target.getY(), 90.0)
                            robot.TurnGoToPosition(target, .5, 2.0, this)
                            target.setCoordinate(-60 * direction, 84.0, 90.0)
                        }
                    }

                // -These are all paths where we move the Foundation or no one moves the Foundation-
                }else {
                    for (i in (0..totalStones)) { // Cycle each stone to the unmoved foundation
                        target.setCoordinate(-42 * direction , stoneOrder[i] * 8 + 13.toDouble(), -90.0)
                        robot.StraightGoToPosition(target, .45, 1.0, this)
                        grab(direction)
                        if (foundation == Foundation.YES && i == totalStones - 1) {
                            target.setCoordinate(-45 * direction, 134.0, -90.0)
                        }else {
                            target.setCoordinate(-45 * direction, 96.0, -90.0)
                            robot.StraightGoToPosition(target, .47, 1.0, this)
                            drop(direction)
                        }
                    }
                    robot.setSpeedZero()
                    if (foundation == Foundation.YES) { // We move the Foundation
                        target.setCoordinate(target.getX(), target.getY(), angle + 180)
                        robot.TurnGoToPosition(target, 1.0, 2.0, this)
                        target.setCoordinate(target.getX() + (9 * direction), target.getY(), angle + 180)
                        robot.StraightGoToPosition(target, .3, .7, this)
                        robot.closeFoundation()
                        sleep(500)
                        target.setCoordinate(target.getX(), target.getY(), 90.0)
                        robot.DoGoToPosition(
                                target,
                                1.0,
                                KPID(.01, 0.0, 0.0),
                                KPID(5.0, 0.0, 0.0),
                                8.0,
                                30.0,
                                KRoboMovement.State.INIT,
                                this)
                        target.setCoordinate(target.getX(), target.getY(), angle  + (direction * -10))
                        robot.DoGoToPosition(
                                target,
                                1.0,
                                KPID(.01, 0.0,0.0),
                                KPID(5.0, 0.0, 0.0),
                                8.0,
                                10.0,
                                KRoboMovement.State.INIT,
                                this)
                        robot.openFoundation()
                        sleep(500)
                        target.setCoordinate(robot.getX(), robot.getY() - 12, angle)
                        robot.StraightGoToPosition(target, .7, 5.0, this)
                        target.setCoordinate(target.getX() + (-13 * direction), target.getY() + 6, angle)
                        robot.StraightGoToPosition(target, .4, 1.0, this)
                    }
                    if (park == Park.BRIDGE) { // Park next to Bridge
                        target.setCoordinate(-43 * direction, 72.0, angle)
                        robot.StraightGoToPosition(target, .71, 6.0, this)
                        robot.setSpeedZero()
                    }
                    else if (park == Park.WALL) { // Park next to Wall
                        target.setCoordinate(-62 * direction, 72.0, angle)
                        robot.StraightGoToPosition(target, 1.0, 2.0, this)
                        robot.setSpeedZero()
                    }
                    stop()
                }
            }
        }
    }

    /**
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    /**
     * For Debug mode, runs a particular task.
     * @param taskname The name of the task to be run.
     * @param debug The actual runmode.
     * @param default_value Whether or not to run this method when in auto mode.
     * @return Whether or not to run this segment.
     */
    fun DoTask(taskname: String, debug: RunMode, default_value: Boolean): Boolean {
        dashboard?.displayPrintf(0, taskname)
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard?.displayPrintf(1, "Press A to run, B to skip")
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    dashboard?.displayPrintf(1, "Run")
                    return true
                }
                if (gamepad1.b) {
                    dashboard?.displayPrintf(1, "Skip")
                    sleep(1000)
                    return false
                }
            }
        }
        return default_value
    }

    /**
     * Drives forwards, picks up the stone, then drives backwards.
     * @param direction Determines which direction to drive.
     */
    fun grab(direction: Double) {
        target.setCoordinate(target.getX() + (direction * 6), target.getY(),
                Math.toDegrees(target.getAngle()))
        robot.StraightGoToPosition(target, .1, .4, this)
        if (alliance == Alliance.BLUE)
            grabStoneR()
        else
            grabStoneL()
        target.setCoordinate(target.getX() - (direction * 8), target.getY(),
                Math.toDegrees(target.getAngle()))
        robot.StraightGoToPosition(target, .2, .5, this)
    }

    /**
     * Drives forwards, drops off stone, then drives backwards.
     * @param direction Determines which direction to drive.
     */
    fun drop(direction: Double) {
        target.setCoordinate(target.getX() + (7.5 * direction), target.getY(),
                Math.toDegrees(target.getAngle()))
        robot.StraightGoToPosition(target, .15, .5, this)
        if (alliance == Alliance.BLUE)
            dropStoneR()
        else
            dropStoneL()
        target.setCoordinate(target.getX() - (5.5 * direction), target.getY(),
                Math.toDegrees(target.getAngle()))
        robot.StraightGoToPosition(target, .2, .5, this)
    }

    /**
     * Grab a stone with the right claw
     */
    fun grabStoneR() {
        robot.rightFinger?.setPosition(KOlivanieV3Hardware.FINGERRELEASER)
        sleep(500)
        robot.rightSideClaw?.setPosition(KOlivanieV3Hardware.GRABBEDR)
        sleep(500)
        robot.rightFinger?.setPosition(KOlivanieV3Hardware.FINGERGRABBEDR)
        sleep(500)
        robot.rightSideClaw?.setPosition(KOlivanieV3Hardware.RELEASEDR + .2)
    }

    /**
     * Drop a stone with the right claw
     */
    fun dropStoneR() {
        robot.rightSideClaw?.setPosition(KOlivanieV3Hardware.GRABBEDR)
        sleep(500)
        robot.rightFinger?.setPosition(KOlivanieV3Hardware.FINGERRELEASER)
        sleep(250)
        robot.rightSideClaw?.setPosition(KOlivanieV3Hardware.RELEASEDR)
        sleep(500)
        robot.rightFinger?.setPosition(KOlivanieV3Hardware.FINGERGRABBEDR)
    }

    /**
     * Grab a stone with the left claw
     */
    fun grabStoneL() {
        robot.leftFinger?.setPosition(KOlivanieV3Hardware.FINGERRELEASEL)
        sleep(500)
        robot.leftSideClaw?.setPosition(KOlivanieV3Hardware.GRABBEDL)
        sleep(500)
        robot.leftFinger?.setPosition(KOlivanieV3Hardware.FINGERGRABBEDL)
        sleep(500)
        robot.leftSideClaw?.setPosition(KOlivanieV3Hardware.RELEASEDL - .2)
    }

    /**
     * Drop a stone with the left claw
     */
    fun dropStoneL() {
        robot.leftSideClaw?.setPosition(KOlivanieV3Hardware.GRABBEDL)
        sleep(500)
        robot.leftFinger?.setPosition(KOlivanieV3Hardware.FINGERRELEASEL)
        sleep(250)
        robot.leftSideClaw?.setPosition(KOlivanieV3Hardware.RELEASEDL)
        sleep(500)
        robot.leftFinger?.setPosition(KOlivanieV3Hardware.FINGERGRABBEDL)
    }

    // SKYSTONE-------------------------------------------------------------------------------------


    //detection pipeline
    class StageSwitchingPipeline: OpenCvPipeline() {
        var yCbCrChan2Mat: Mat= Mat()
        var thresholdMat: Mat= Mat()
        var all: Mat= Mat()
        var contoursList = mutableListOf<MatOfPoint>()

        enum class Stage {
//          color difference. greyscale
            detection, //includes outlines
            THRESHOLD, //b&w
            RAW_IMAGE, //displays raw view
        }

        private var stageToRenderToViewport: Stage= Stage.detection
        private var stages= Stage.values()


        override fun onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            var currentStageNum: Int= stageToRenderToViewport.ordinal

            var nextStageNum: Int = currentStageNum + 1

            if(nextStageNum >= stages.size) {
                nextStageNum = 0
            }

            stageToRenderToViewport = stages[nextStageNum]
        }

        override fun processFrame(input: Mat): Mat? {
            contoursList.clear()

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb) //converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2)    //takes cb difference and
            // stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102.0, 255.0,
                    Imgproc.THRESH_BINARY_INV)

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, Mat(), Imgproc.RETR_LIST,
                    Imgproc.CHAIN_APPROX_SIMPLE)
            yCbCrChan2Mat.copyTo(all)   //copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue
            // contours


            //get values from frame
            var pixMid: DoubleArray   //gets value at circle
            var pixLeft: DoubleArray  //gets value at circle
            var pixRight: DoubleArray //gets value at circle
            var midSum: Double= 0.0
            var leftSum: Double= 0.0
            var rightSum: Double= 0.0
//            var i: Int= 0

            for(i in (-10 .. 10)){
                for(j in (-10 .. 10)) {
                    pixMid = thresholdMat.get((input.rows()* midPos[1]).toInt() + i, (input.cols()* midPos[0]).toInt() + j)
                    pixLeft = thresholdMat.get((input.rows()* leftPos[1]).toInt() + i, (input.cols()* leftPos[0]).toInt() + j)
                    pixRight = thresholdMat.get((input.rows()* rightPos[1]).toInt() + i, (input.cols()* rightPos[0]).toInt() + j)
                    midSum += pixMid[0]
                    leftSum += pixLeft[0]
                    rightSum += pixRight[0]
                }
            }

            valMid = midSum / 400
            valLeft = leftSum / 400
            valRight = rightSum / 400

            //create three points
            var pointMid: Point= Point((input.cols()* midPos[0]).toDouble(), (input.rows() * midPos[1]).toDouble())
            var pointLeft: Point= Point((input.cols()* leftPos[0]).toDouble(), ((input.rows() * leftPos[1])).toDouble())
            var pointRight: Point= Point((input.cols()* rightPos[0]).toDouble(), (input.rows() * rightPos[1]).toDouble())

            //draw circles on those points
            Imgproc.circle(all, pointMid,10, Scalar( 255.0, 0.0, 0.0 ),1 )
            Imgproc.circle(all, pointLeft,10, Scalar(255.0, 0.0, 0.0 ),1 )
            Imgproc.circle(all, pointRight,10, Scalar( 255.0, 0.0, 0.0 ),1 )

//            draw 3 rectangles
//            1-3
            Imgproc.rectangle(
                    all,
                    Point(input.cols()*(leftPos[0])-10.toDouble(), (input.rows()*(leftPos[1])-10).toDouble()),
                    Point(input.cols()*(leftPos[0])+10.toDouble(), (input.rows()*(leftPos[1])+10).toDouble()),
                    Scalar(0.0, 255.0, 255.0),
                    3)

//            3-5
            Imgproc.rectangle(
                    all,
                    Point(input.cols()*(midPos[0])-10.toDouble(), (input.rows()*(midPos[1])-10).toDouble()),
                    Point(input.cols()*(midPos[0])+10.toDouble(), input.rows()*(midPos[1])+10.toDouble()),
                    Scalar(0.0, 255.0, 255.0),
                    3)
//            5-7
            Imgproc.rectangle(
                    all,
                    Point(input.cols()*(rightPos[0])-10.toDouble(), input.rows()*(rightPos[1])-10.toDouble()),
                    Point(input.cols()*(rightPos[0])+10.toDouble(), input.rows()*(rightPos[1])+10.toDouble()),
                    Scalar(0.0, 255.0, 255.0),
                    3)

//            draw 3 rectangles
//            1-3
            Imgproc.rectangle(
                    all,
                    Point(input.cols() * (leftPos[0]-rectWidth/2).toDouble(), input.rows()*(leftPos[1]-rectHeight/2).toDouble()),
                    Point(input.cols()*(leftPos[0]+rectWidth/2).toDouble(), input.rows()*(leftPos[1]+rectHeight/2).toDouble()),
                    Scalar(0.0, 255.0, 0.0),
                    3)

//            3-5
            Imgproc.rectangle(
                    all,
                    Point(input.cols()*(midPos[0]-rectWidth/2).toDouble(), input.rows()*(midPos[1]-rectHeight/2).toDouble()),
                    Point(input.cols()*(midPos[0]+rectWidth/2).toDouble(), input.rows()*(midPos[1]+rectHeight/2).toDouble()),
                    Scalar(0.0, 255.0, 0.0),
                    3)

//            5-7
            Imgproc.rectangle(
                    all,
                    Point(input.cols()*(rightPos[0]-rectWidth/2).toDouble(), input.rows()*(rightPos[1]-rectHeight/2).toDouble()),
                    Point(input.cols()*(rightPos[0]+rectWidth/2).toDouble(), input.rows()*(rightPos[1]+rectHeight/2).toDouble()),
                    Scalar(0.0, 255.0, 0.0),
                    3)

            return when (stageToRenderToViewport) {

                Stage.THRESHOLD -> thresholdMat

                Stage.detection -> all

                Stage.RAW_IMAGE -> input

            }
        }

    }

    // MENU ----------------------------------------------------------------------------------------
    override fun isMenuUpButton(): Boolean { return gamepad1.dpad_up }

    override fun isMenuDownButton(): Boolean { return gamepad1.dpad_down }

    override fun isMenuEnterButton(): Boolean { return gamepad1.dpad_right }

    override fun isMenuBackButton(): Boolean { return gamepad1.dpad_left }


    private fun doMenus() {

        var modeMenu: FtcChoiceMenu<RunMode> = FtcChoiceMenu("Run Mode", null, this)
        var delayMenu: FtcValueMenu = FtcValueMenu("Delay:", modeMenu, this, 0.0, 20000.0, 1000.0, 0.0, "%.0f msec")
        var allianceMenu: FtcChoiceMenu<Alliance> =  FtcChoiceMenu("Alliance:", delayMenu, this)
        var startPositionMenu: FtcChoiceMenu<StartPosition> = FtcChoiceMenu("Start Position:", allianceMenu, this)
        var skystonesMenu: FtcChoiceMenu<Skystones> = FtcChoiceMenu("Number of Skystones:", startPositionMenu, this)
        var stonesMenu: FtcChoiceMenu<Stones> = FtcChoiceMenu("Number of Stones:", skystonesMenu, this)
        var foundationMenu: FtcChoiceMenu<Foundation> = FtcChoiceMenu("Move the Foundation:", stonesMenu, this)
        var parkMenu: FtcChoiceMenu<Park> =  FtcChoiceMenu("Park :", foundationMenu, this)

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
        if (skystonesMenu.getCurrentChoiceObject() == Skystones.ONE || skystonesMenu.getCurrentChoiceObject() == Skystones.ZERO) {
            stonesMenu.addChoice("Five", Stones.FIVE, false, foundationMenu)
        }

        if (skystonesMenu.getCurrentChoiceObject() == Skystones.ZERO) {
            stonesMenu.addChoice("Six", Stones.SIX, false, foundationMenu)
        }

        foundationMenu.addChoice("Yes", Foundation.YES, true, parkMenu)
        foundationMenu.addChoice("No", Foundation.NO, false, parkMenu)
        foundationMenu.addChoice("Partner does it", Foundation.PARTNER, false,
                parkMenu)

        parkMenu.addChoice("Wall", Park.WALL, true)
        parkMenu.addChoice("Bridge", Park.BRIDGE, false)
        parkMenu.addChoice("None", Park.NONE, false)


        FtcMenu.walkMenuTree(modeMenu, this)
        runmode = modeMenu.getCurrentChoiceObject()
        delay = delayMenu.getCurrentValue().toInt()
        alliance = allianceMenu.getCurrentChoiceObject()
        startposition = startPositionMenu.getCurrentChoiceObject()
        skystones = skystonesMenu.getCurrentChoiceObject()
        stones = stonesMenu.getCurrentChoiceObject()
        foundation = foundationMenu.getCurrentChoiceObject()
        park = parkMenu.getCurrentChoiceObject()

        dashboard?.displayPrintf(13, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(),
                runmode.toString())
        dashboard?.displayPrintf(14, "Delay = %d msec", delay)
        dashboard?.displayPrintf(15, "Alliance: %s (%s)",
                allianceMenu.getCurrentChoiceText(), alliance.toString())
        dashboard?.displayPrintf(16, "Start position: %s (%s)",
                startPositionMenu.getCurrentChoiceText(), startposition.toString())
        dashboard?.displayPrintf(17, "Number of Skystones: %s (%s)",
                skystonesMenu.getCurrentChoiceText(), skystones.toString())
        dashboard?.displayPrintf(18, "Number of Stones: %s (%s)",
                stonesMenu.getCurrentChoiceText(), stones.toString())
        dashboard?.displayPrintf(19, "Move Foundation: %s (%s)",
                foundationMenu.getCurrentChoiceText(), foundation.toString())
        dashboard?.displayPrintf(20, "Park: %s (%s)", parkMenu.getCurrentChoiceText(),
                park.toString())
    }
    // END MENU ------------------------------------------------------------------------------------
}
