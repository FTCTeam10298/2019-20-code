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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@Autonomous(name="Olivanie Autonomous Linear", group ="Olivanie")
public class Olivanie_Autonomous_Linear extends LinearOpMode implements FtcMenu.MenuButtons {
    public enum RunMode {
        RUNMODE_AUTO,
        RUNMODE_DEBUG
    }

    public enum Alliance {
        BLUE,
        RED
    }

    // Building Zone vs Loading Zone. 1 is further from the audience.
    public enum StartPosition {
        BUILDING1,
        BUILDING2,
        LOADING1,
        LOADING2
    }

    public enum Skystones {
        ZERO,
        ONE,
        TWO
    }

    public enum SkystonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum Stones {
        ZERO,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX
    }

    public enum Foundation {
        IN_FRONT_OF_US,
        BACK_WALL,
        NO,
        PARTNER
    }

    public enum Park {
        WALL,
        BRIDGE,
        NONE
    }

    public enum Auto_path {
        PATH1,
        PATH2,
        PATH3,
        PATH4
    }

    // Menu option variables
    RunMode runmode = RunMode.RUNMODE_AUTO;
    int delay = 0;
    Alliance alliance = Alliance.RED;
    StartPosition startposition = StartPosition.LOADING2;
    Skystones skystones = Skystones.ONE;
    SkystonePosition skystonePosition = SkystonePosition.CENTER;
    Stones stones = Stones.ZERO;
    Foundation foundation = Foundation.IN_FRONT_OF_US;
    Park park = Park.WALL;
    Auto_path path = Auto_path.PATH1;

    // Order of Stones depending on the Skystone
    int[] stoneOrderLB = {5, 2, 4, 3, 1, 0};
    int[] stoneOrderC = {4, 1, 5, 3, 2, 0};
    int[] stoneOrderRB = {3, 0, 5, 4, 2, 1};
    int[] stoneOrderN = {5, 4, 3, 2, 1, 0};

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    RoboMovement robot = new RoboMovement();
    Coordinate target = new Coordinate();

    // Skystone variables
    private ElapsedTime runtime = new ElapsedTime();


    double time_a       = 0;
    double dt           = 0;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0
    // or 255

    private static double valMid = 0;
    private static double valLeft = 0;
    private static double valRight = 0;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles
    // left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or
    // down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;


    static final double COUNTS_PER_MOTOR_REV  = 28.0;      // Rev HD Hex v2.1 Motor encoder
    static final double GEARBOX_RATIO         = 20.0;      // 40 for 40:1, 20 for 20:1
    static final double DRIVE_GEAR_REDUCTION  = 1;         // This is > 1.0 if geared for torque
    static final double WHEEL_DIAMETER_INCHES = 3.937007874015748; // For figuring circumference
    static final double DRIVETRAIN_ERROR      = 1.1;       // Error determined from testing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO *
            DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR;
    static final double COUNTS_PER_DEGREE     = COUNTS_PER_INCH*0.184; // Found by testing

    int multipier = 1;

    @Override
    public void runOpMode() {
        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap);

        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry);



        // Run though the menu ---------------------------------------------------------------------
        doMenus();

        dashboard.displayPrintf(1, "For a little bot, I pack a biiiiiig punch!");


        if (alliance == Alliance.RED) {
            multipier = -1;
            if (startposition == StartPosition.BUILDING1) {
                if (skystones == Skystones.ZERO) {
                    if (stones == Stones.ZERO) {
                        if (foundation == Foundation.IN_FRONT_OF_US) {
                            if (park == Park.WALL) {
                                path = Auto_path.PATH1;
                            }
                            else if (park == Park.BRIDGE) {
                                path = Auto_path.PATH3;
                            }
                        }
                    }
                }
            }
        } else if (alliance == Alliance.BLUE) {
            if (startposition == StartPosition.BUILDING1) {
                if (skystones == Skystones.ZERO) {
                    if (stones == Stones.ZERO) {
                        if (foundation == Foundation.IN_FRONT_OF_US) {
                            if (park == Park.WALL) {
                                path = Auto_path.PATH2;
                            }
                            else if (park == Park.BRIDGE) {
                                path = Auto_path.PATH4;
                            }
                        }
                    }
                }
            }
        }


        double angle;
        double direction;

        if (alliance == Alliance.RED) {
            angle = 180;
            direction = -1;
        }
        else {
            angle = 0;
            direction = 1;
        }


        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        // Sample while waiting for start
        while (!opModeIsActive() && !isStopRequested()) {
            dashboard.displayPrintf(3, "Left Value: (%f)", valLeft);
            dashboard.displayPrintf(4, "Middle Value: (%f)", valMid);
            dashboard.displayPrintf(5, "Right Value: (%f)", valRight);
            dashboard.displayPrintf(6, "Height: ($d)", rows);
            dashboard.displayPrintf(7, "Width: (%d)", cols);
            if (valLeft < valMid && valLeft < valRight) {
                dashboard.displayPrintf(9, "Skystone is LEFT");
                skystonePosition = SkystonePosition.LEFT;
            }
            else if (valMid < valLeft && valMid < valRight) {
                dashboard.displayPrintf(9, "Skystone is CENTER");
                skystonePosition = SkystonePosition.CENTER;
            }
            else {
                dashboard.displayPrintf(9, "Skystone is RIGHT");
                skystonePosition = SkystonePosition.RIGHT;
            }

            telemetry.update();
            sleep(100);
        }
        //webcam.stopStreaming();
        //webcam.closeCameraDevice();

        // Set starting location based off of menu options
        if (alliance == Alliance.BLUE) {
            if (startposition == StartPosition.LOADING2) {
                robot.setGlobalRobot(-63, 36, 0);
                //robot.setGlobalRobot(0, 0, 0);
            }
            else if (startposition == StartPosition.LOADING1)
                robot.setGlobalRobot(-63, 60, 0);
            else if (startposition == StartPosition.BUILDING2)
                robot.setGlobalRobot(-63, 84, 0);
            else
                robot.setGlobalRobot(-63, 108, 0);
        }
        else {
            if (startposition == StartPosition.LOADING2)
                robot.setGlobalRobot(63, 36, -Math.PI);
            else if (startposition == StartPosition.LOADING1)
                robot.setGlobalRobot(63, 60, -Math.PI);
            else if (startposition == StartPosition.BUILDING2)
                robot.setGlobalRobot(63, 84, -Math.PI);
            else
                robot.setGlobalRobot(63, 108, -Math.PI);
        }

        // Determine the order of stones depending on the Skystone location
        int[] stoneOrder = stoneOrderLB;
        if (skystonePosition == SkystonePosition.CENTER) {
            stoneOrder = stoneOrderC;
        }
        else if ((alliance == Alliance.BLUE && skystonePosition == SkystonePosition.RIGHT) ||
                (alliance == Alliance.RED && skystonePosition == SkystonePosition.LEFT)) {
            stoneOrder = stoneOrderRB;
        }
        if (skystones == Skystones.ZERO)
            stoneOrder = stoneOrderN;

        // Find the total amount of stones being cycled
        double totalStones;
        if (stones == Stones.ZERO)
            totalStones = 0;
        else if (stones == Stones.ONE)
            totalStones = 1;
        else if (stones == Stones.TWO)
            totalStones = 2;
        else if (stones == Stones.THREE)
            totalStones = 3;
        else if (stones == Stones.FOUR)
            totalStones = 4;
        else if (stones == Stones.FIVE)
            totalStones = 5;
        else
            totalStones = 6;
        if (skystones == Skystones.ONE)
            totalStones += 1;
        else if (skystones == Skystones.TWO)
            totalStones += 2;

        double speed = 0;

        if (totalStones == 0 || totalStones == 1)
            speed = .5;
        else if (totalStones == 2)
            speed = .85;
        else
            speed = 1;

        /**
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running");

//        // For testing odometry code
//        if (DoTask("Odometry Test", runmode, false)) {
//            dashboard.displayPrintf(2, "%f", robot.getX());
//            dashboard.displayPrintf(3, "%f", robot.getY());
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
        sleep(delay);
        if (DoTask("Test", runmode, false)) {
//            robot.setGlobalRobot(0,0,0);
//            for (int i = 0; i < 2; i++) {
//                target.setCoordinate(target.getX(), target.getY() + 72, target.getAngle());
//                robot.StraightGoToPosition(target, .2, .1, this);
//                sleep(5000);
//                target.setCoordinate(target.getX(), target.getY() - 72, target.getAngle());
//                robot.StraightGoToPosition(target, .2, .1, this);
//                sleep(5000);
//
////                target.setCoordinate(0, 0, 135);
////                robot.DoGoToPosition(target, .2, new PID(0, 0, 0),
////                        new PID (.5, 0, 0), 10, 1, RoboMovement.State.INIT, this);
////                sleep(5000);
////                target.setCoordinate(0, 0, 0);
////                robot.DoGoToPosition(target, .2, new PID(0, 0, 0),
////                        new PID (.5, 0, 0), 10, 1, RoboMovement.State.INIT, this);
////                sleep(5000);
//            }
//            grabStoneL();
//            sleep(1000);
//            dropStoneL();
//            sleep(1000);
//            grabStoneR();
//            sleep(1000);
//            dropStoneR();
//            sleep(1000);

//            robot.setGlobalRobot(0, 0, Math.toRadians(-90));
//            target.setCoordinate(0, 0, -90);
//
//            grab(-1); //red
//            sleep(1000);
//            grab(1); //blue

//
            target.setCoordinate(robot.getX(), robot.getY(), Math.toDegrees(robot.getWorldAngle_rad()));

            target.setCoordinate(target.getX(), target.getY() - 72, target.getAngle());
            robot.StraightGoToPosition(target, .5, 1, this);
            sleep(2000);
            target.setCoordinate(target.getX(), target.getY() + 72, target.getAngle());
            robot.StraightGoToPosition(target, .5, 1, this);
            sleep(2000);
            target.setCoordinate(target.getX(), target.getY() - 72, target.getAngle());
            robot.StraightGoToPosition(target, .5, 1, this);
            sleep(2000);
            target.setCoordinate(target.getX(), target.getY() + 72, target.getAngle());
            robot.StraightGoToPosition(target, .5, 1, this);
            sleep(2000);

//            target.setCoordinate(-42 * direction , 5 * 8 + 13,
//                    -90);
//            robot.StraightGoToPosition(target, .45, 1,
//                    this);

//            robot.driveSetPower(.5, .5, .5, .5);
//            sleep(3000);
//            robot.setSpeedZero();
        }

        // Init
        if (DoTask("Init", runmode, true)) {
        }

        if (DoTask("Drive", runmode, true)) {
            // --------------------------These are all paths without stones-------------------------
            if (totalStones == 0) {
                // --------These are all paths that only move the foundation in front of us---------
                if (foundation == Foundation.IN_FRONT_OF_US) {
                    target.setCoordinate(-28 * direction, 126, angle + 180);
                    robot.StraightGoToPosition(target, 1, 1,
                            this);
                    robot.closeFoundation();
                    sleep(500);
                    target.setCoordinate(-60 * direction, 126, angle + 180);
                    robot.StraightGoToPosition(target, 1, 1,
                            this);
                    robot.openFoundation();
                    sleep(500);
                    target.setCoordinate(-60 * direction, 84, angle + 180);
                    robot.StraightGoToPosition(target, 1, 2,
                            this);
                    if (park == Park.WALL) // We Park by Wall
                        target.setCoordinate(-60 * direction, 72, angle + 180);
                    else if (park == Park.BRIDGE) //We Park by Bridge
                        target.setCoordinate(-36 * direction, 72, angle + 180);
                    robot.StraightGoToPosition(target, 1, 1,
                            this);
                    robot.setSpeedZero();
                }
                // -----These are all paths that only move the foundation against the back wall-----
                else if (foundation == Foundation.BACK_WALL) {
                    target.setCoordinate(-28 * direction, 126, angle + 180);
                    robot.StraightGoToPosition(target, 1, 1,
                            this);
                    robot.closeFoundation();
                    sleep(500);
                }
                // -------------------------These are paths that only park--------------------------
                else {
                    // Parking against the wall
                    if (park == Park.WALL) {
                        target.setCoordinate(-63 * direction, 72, angle);
                        robot.StraightGoToPosition(target, 1, 2,
                                this);
                    }
                    // Parking against the bridge
                    else if (park == Park.BRIDGE) { // We Park by the Bridge
                        target.setCoordinate(-36 * direction, 72, angle);
                        robot.StraightGoToPosition(target, 1, 2,
                                this);
                    }
                    robot.setSpeedZero();
                }
            }
            // ----------------------------These are all paths with stones--------------------------
            else {
                // ------------These are all paths where our partner moves the Foundation-----------
                if (foundation == Foundation.PARTNER) {
                    for (int i = 0; i < totalStones; i++) {
                        if (i == 0)
                            target.setCoordinate(-40 * direction , stoneOrder[i] * 8 + 10,
                                    -90);
                        else
                            target.setCoordinate(-40 * direction , stoneOrder[i] * 8 + 10,
                                    -90);
                        robot.StraightGoToPosition(target, speed, 1,
                                this);
                        grab(direction);
                        target.setCoordinate(target.getX(), target.getY(), 90);
                        robot.TurnGoToPosition(target, speed,  2,
                                this);
                        target.setCoordinate(-60 * direction, 84, 90);
                    }
                }
                // -These are all paths where we move the Foundation or no one moves the Foundation-
                else {
                    for (int i = 0; i < totalStones; i++){ // Cycle each stone to the unmoved
                        // Foundation
                        if (i != 0) {
                            target.setCoordinate(-41 * direction, 72, -90);
                            robot.StraightGoToPosition(target, speed, 3,
                                    this);
                        }
                        if (direction == 1) {
                            robot.leftFinger.setPosition(robot.FINGERRELEASEL);
                            robot.leftSideClaw.setPosition(robot.DROPPEDL);
                        }
                        else {
                            robot.rightFinger.setPosition(robot.FINGERRELEASER);
                            robot.rightSideClaw.setPosition(robot.DROPPEDR);
                        }
                        target.setCoordinate(-39 * direction , stoneOrder[i] * 8 + 9,
                                      -90);
                        robot.StraightGoToPosition(target, speed, 1, this);
                        grab(direction);
                        target.setCoordinate(-41 * direction, 90, -90);
                        robot.StraightGoToPosition(target, speed, 10, this);
                        if ((foundation == Foundation.IN_FRONT_OF_US || foundation ==
                                Foundation.BACK_WALL) && i == totalStones - 1)
                            target.setCoordinate(-30 * direction, 120, -90);
                        else
                            target.setCoordinate(-30 * direction, 130, -90);
                        robot.StraightGoToPosition(target, speed,  1, this);
                        drop(direction);
                    }
                    robot.setSpeedZero();
                    if (foundation == Foundation.IN_FRONT_OF_US) { // We move the Foundation
//                        target.setCoordinate(-36 * direction, 120, angle - 180);
//                        robot.StraightGoToPosition(target, speed,  1,
//                                this);
//                        target.setCoordinate(-27 * direction, 120, angle - 180);
//                        robot.StraightGoToPosition(target, .3,  1,
//                                this);
//                        robot.closeFoundation();
//                        sleep(500);
//                        target.setCoordinate(-40 * direction, 120, angle - 180);
//                        robot.StraightGoToPosition(target, speed,  1,
//                                this);
//                        target.setCoordinate(-40 * direction, 120, 90);
//                        robot.DoGoToPosition(target, 1, new PID(1, 0, 0),
//                                new PID(20, 0, 0), 4, 5,
//                                RoboMovement.State.INIT, this);
//                        target.setCoordinate(-40 * direction, 120, angle);
//                        robot.DoGoToPosition(target, 1, new PID(1, 0, 0),
//                                new PID(20, 0, 0), 4, 5,
//                                RoboMovement.State.INIT, this);
//                        robot.openFoundation();
//                        sleep(500);
//                        target.setCoordinate(-33 * direction, 120, angle);
//                        robot.StraightGoToPosition(target, .6, 1, this);
//                        robot.closeFoundation();
//                        sleep(500);
//                        target.setCoordinate(-50 * direction, 120, angle);
//                        robot.StraightGoToPosition(target, speed, 2, this);
////                        target.setCoordinate(target.getX(), target.getY(), 90);
////                        robot.DoGoToPosition(target, 1, new PID(.01, 0,0),
////                                new PID(5, 0, 0), 8, 30,
////                                RoboMovement.State.INIT, this);
////                        target.setCoordinate(target.getX(), target.getY(), angle  +
////                                (direction * -10));
////                        robot.DoGoToPosition(target, 1, new PID(.01, 0,0),
////                                new PID(5, 0, 0), 8, 10,
////                                RoboMovement.State.INIT, this);
////                        robot.openFoundation();
////                        sleep(500);
////                        target.setCoordinate(robot.getX(), robot.getY() - 12, angle);
////                        robot.StraightGoToPosition(target, .7, 5,
////                                this);
////                        target.setCoordinate(target.getX() + (-13 * direction),
////                                target.getY() + 6, angle);
////                        robot.StraightGoToPosition(target, .4, 1,
////                                this);
                        target.setCoordinate(-35 * direction, 120, angle - 180);
                        robot.StraightGoToPosition(target, speed,  2,
                                this);
                        target.setCoordinate(-25.5 * direction, 120, angle - 180);
                        robot.StraightGoToPosition(target, .35,  2,
                                this);
                        robot.closeFoundation();
                        sleep(490);
                        target.setCoordinate(-60 * direction, 125, angle - 180);
                        robot.DoGoToPosition(target, 1, new PID(1, 0, 0),
                                new PID(20, 0, 0), 2, 10,
                                RoboMovement.State.INIT, this);
                        robot.openFoundation();
                        sleep(390);
                        target.setCoordinate(-61 * direction, 90, -angle - 180);
                        robot.StraightGoToPosition(target, speed,  2,
                                this);
                    }
                    else if (foundation == Foundation.BACK_WALL) {
                        target.setCoordinate(-35 * direction, 120, angle - 180);
                        robot.DoGoToPosition(target, speed, new PID(.1, 0, 0),
                                new PID(2, 0, 0), 2, 10,
                                RoboMovement.State.INIT, this);
                        target.setCoordinate(-25.5 * direction, 120, angle - 180);
                        robot.StraightGoToPosition(target, .35,  2,
                                this);
                        robot.closeFoundation();
                        sleep(490);
                        if (alliance == Alliance.BLUE)
                            target.setCoordinate(-46, 110, -100);
                        else
                            target.setCoordinate(46, 110, -100);
                        robot.DoGoToPosition(target, 1, new PID(1, 0, 0),
                                new PID(20, 0, 0), 4, 10,
                                RoboMovement.State.INIT, this);
                        robot.openFoundation();
                        sleep(390);
                        target.setCoordinate(-46 * direction, 113, -90);
                        robot.DoGoToPosition(target, 1, new PID(1, 0, 0),
                                new PID(20, 0, 0), 2, 5,
                                RoboMovement.State.INIT, this);
                    }
                    if (park == Park.BRIDGE) { // Park next to Bridge
                        target.setCoordinate(-40 * direction, 72, -90);
                        robot.StraightGoToPosition(target, .85, 6,
                                this);
                        robot.setSpeedZero();
                    }
                    else if (park == Park.WALL) { // Park next to Wall
                        target.setCoordinate(-62 * direction, 72, -90);
                        robot.StraightGoToPosition(target, 1, 2,
                                this);
                        robot.setSpeedZero();
                    }
                    stop();
                }
            }
        }
    }

    /**
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    /**
     * Runs a task, which has no effect in Auto mode.
     * Tasks are chunks of the program that can be separated out.
     * In Debug mode, this runs a particular task.
     * @param taskname The name of the task to be run.
     * @param debug The actual runmode.
     * @param default_value Whether or not to run this method when in auto mode.
     * @return Whether or not to run this segment.
     */
    boolean DoTask(String taskname, RunMode debug, boolean default_value)
    {
        dashboard.displayPrintf(0, taskname);
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard.displayPrintf(1, "Press A to run, B to skip");
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    dashboard.displayPrintf(1, "Run");
                    return true;
                }
                if (gamepad1.b) {
                    dashboard.displayPrintf(1, "Skip");
                    sleep(1000);
                    return false;
                }
            }
        }
        return default_value;
    }

    /**
     * Drives sideways, picks up the stone, then drives backwards.
     * @param direction Determines which direction to drive.
     */
    public void grab (double direction) {
        target.setCoordinate(target.getX() + (direction * 4.5), target.getY(),
                Math.toDegrees(target.getAngle()));
        robot.StraightGoToPosition(target, .4, 2.2, this);
        if (direction == 1) {
            robot.leftFinger.setPosition(robot.FINGERGRABBEDL);
            sleep(620);
            robot.leftSideClaw.setPosition(robot.HELDL);
        }
        else {
            robot.rightFinger.setPosition(robot.FINGERGRABBEDR);
            sleep(620);
            robot.rightSideClaw.setPosition(robot.HELDR);
        }
    }

    /**
     * Drives forwards, drops off stone, then drives backwards.
     * @param direction Determines which direction to drive.
     */
    public void drop (double direction) {
//        target.setCoordinate(target.getX() + (7.5 * direction), target.getY(),
//                Math.toDegrees(target.getAngle()));
//        robot.StraightGoToPosition(target, .15, .5, this);
//        if (alliance == Alliance.BLUE)
//            dropStoneR();
//        else
//            dropStoneL();
//        target.setCoordinate(target.getX() - (5.5 * direction), target.getY(),
//                Math.toDegrees(target.getAngle()));
//        robot.StraightGoToPosition(target, .2, .5, this);
        if (direction == 1) {
            robot.leftSideClaw.setPosition(robot.DROPPEDL);
            sleep(300);
            robot.leftFinger.setPosition(robot.FINGERRELEASEL);
            sleep(400);
            robot.leftSideClaw.setPosition(robot.HELDL);
            sleep(25);
        }
        else {
            robot.rightSideClaw.setPosition(robot.DROPPEDR);
            sleep(300);
            robot.rightFinger.setPosition(robot.FINGERRELEASER);
            sleep(400);
            robot.rightSideClaw.setPosition(robot.HELDR);
            sleep(25);
        }
    }

    /**
     * Grab a stone with the right claw
     */
    @Deprecated
    public void grabStoneR () {
        robot.rightFinger.setPosition(robot.FINGERRELEASER);
        sleep(500);
        robot.rightSideClaw.setPosition(robot.DROPPEDR);
        sleep(500);
        robot.rightFinger.setPosition(robot.FINGERGRABBEDR);
        sleep(500);
        robot.rightSideClaw.setPosition(robot.HELDR + .2);
    }

    /**
     * Drop a stone with the right claw
     */
    @Deprecated
    public void dropStoneR () {
        robot.rightSideClaw.setPosition(robot.DROPPEDR);
        sleep(500);
        robot.rightFinger.setPosition(robot.FINGERRELEASER);
        sleep(250);
        robot.rightSideClaw.setPosition(robot.HELDR);
        sleep(500);
        robot.rightFinger.setPosition(robot.FINGERGRABBEDR);
    }

    /**
     * Grab a stone with the left claw
     */
    @Deprecated
    public void grabStoneL () {
        robot.leftFinger.setPosition(robot.FINGERRELEASEL);
        sleep(500);
        robot.leftSideClaw.setPosition(robot.DROPPEDL);
        sleep(500);
        robot.leftFinger.setPosition(robot.FINGERGRABBEDL);
        sleep(500);
        robot.leftSideClaw.setPosition(robot.HELDL - .2);
    }

    /**
     * Drop a stone with the left claw
     */
    @Deprecated
    public void dropStoneL () {
        robot.leftSideClaw.setPosition(robot.DROPPEDL);
        sleep(500);
        robot.leftFinger.setPosition(robot.FINGERRELEASEL);
        sleep(250);
        robot.leftSideClaw.setPosition(robot.HELDL);
        sleep(500);
        robot.leftFinger.setPosition(robot.FINGERGRABBEDL);
    }

    // SKYSTONE-------------------------------------------------------------------------------------


    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and
            // stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255,
                    Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST,
                    Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue
            // contours


            //get values from frame
            double[] pixMid;//gets value at circle
            double[] pixLeft;//gets value at circle
            double[] pixRight;//gets value at circle
            double midSum = 0;
            double leftSum = 0;
            double rightSum = 0;
            for (int i = - 10; i < 10; i++) {
                for (int j = -10; j < 10; j++) {
                    pixMid = thresholdMat.get((int)(input.rows()* midPos[1]) + i,
                            (int)(input.cols()* midPos[0]) + j);
                    pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]) + i,
                            (int)(input.cols()* leftPos[0]) + j);
                    pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]) + i,
                            (int)(input.cols()* rightPos[0]) + j);
                    midSum += pixMid[0];
                    leftSum += pixLeft[0];
                    rightSum += pixRight[0];
                }
            }

            valMid = midSum / 400;
            valLeft = leftSum / 400;
            valRight = rightSum / 400;

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()
                    * midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()
                    * leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()
                    * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,10, new Scalar( 255, 0, 0 ),1 );
            Imgproc.circle(all, pointLeft,10, new Scalar( 255, 0, 0 ),1 );
            Imgproc.circle(all, pointRight,10, new Scalar( 255, 0, 0 ),1 );

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0])-10,
                            input.rows()*(leftPos[1])-10),
                    new Point(
                            input.cols()*(leftPos[0])+10,
                            input.rows()*(leftPos[1])+10),
                    new Scalar(0, 255, 255), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0])-10,
                            input.rows()*(midPos[1])-10),
                    new Point(
                            input.cols()*(midPos[0])+10,
                            input.rows()*(midPos[1])+10),
                    new Scalar(0, 255, 255), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0])-10,
                            input.rows()*(rightPos[1])-10),
                    new Point(
                            input.cols()*(rightPos[0])+10,
                            input.rows()*(rightPos[1])+10),
                    new Scalar(0, 255, 255), 3);


            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

    // MENU ----------------------------------------------------------------------------------------
    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up; }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.dpad_right; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    /**
     * Actually runs the menu for the drivers.
     */
    private void doMenus() {
        FtcChoiceMenu<RunMode> modeMenu = new FtcChoiceMenu<>("Run Mode", null,
                this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay:", modeMenu, this,
                0, 20000, 1000, 0, "%.0f msec");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", delayMenu,
                this);
        FtcChoiceMenu<StartPosition> startPositionMenu = new FtcChoiceMenu<>(
                "Start Position:", allianceMenu, this);
        FtcChoiceMenu<Skystones> skystonesMenu = new FtcChoiceMenu<>(
                "Number of Skystones:", startPositionMenu, this);
        FtcChoiceMenu<Stones> stonesMenu = new FtcChoiceMenu<>("Number of Stones:",
                skystonesMenu, this);
        FtcChoiceMenu<Foundation> foundationMenu = new FtcChoiceMenu<>(
                "Move the Foundation:", stonesMenu, this);
        FtcChoiceMenu<Park> parkMenu = new FtcChoiceMenu<>("Park :", foundationMenu,
                this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, delayMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, delayMenu);

        delayMenu.setChildMenu(allianceMenu);

        allianceMenu.addChoice("Red", Alliance.RED, true, startPositionMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, startPositionMenu);

        startPositionMenu.addChoice("Building Zone Far", StartPosition.BUILDING1,
                false, skystonesMenu);
        startPositionMenu.addChoice("Building Zone Near", StartPosition.BUILDING2,
                false, skystonesMenu);
        startPositionMenu.addChoice("Loading Zone Far", StartPosition.LOADING1,
                false, skystonesMenu);
        startPositionMenu.addChoice("Loading Zone Near", StartPosition.LOADING2,
                true, skystonesMenu);


        skystonesMenu.addChoice("None", Skystones.ZERO, false, stonesMenu);
        skystonesMenu.addChoice("One", Skystones.ONE, true, stonesMenu);
        skystonesMenu.addChoice("Two", Skystones.TWO, false, stonesMenu);

        stonesMenu.addChoice("None", Stones.ZERO, true, foundationMenu);
        stonesMenu.addChoice("One", Stones.ONE, false, foundationMenu);
        stonesMenu.addChoice("Two", Stones.TWO, false, foundationMenu);
        stonesMenu.addChoice("Three", Stones.THREE, false, foundationMenu);
        stonesMenu.addChoice("Four", Stones.FOUR, false, foundationMenu);
        if (skystonesMenu.getCurrentChoiceObject() == Skystones.ONE
                || skystonesMenu.getCurrentChoiceObject() == Skystones.ZERO)
            stonesMenu.addChoice("Five", Stones.FIVE, false, foundationMenu);
        if (skystonesMenu.getCurrentChoiceObject() == Skystones.ZERO)
            stonesMenu.addChoice("Six", Stones.SIX, false, foundationMenu);

        foundationMenu.addChoice("In front of us", Foundation.IN_FRONT_OF_US, true, parkMenu);
        foundationMenu.addChoice("Against the Back Wall", Foundation.BACK_WALL, false, parkMenu);
        foundationMenu.addChoice("No", Foundation.NO, false, parkMenu);
        foundationMenu.addChoice("Partner does it", Foundation.PARTNER, false,
                parkMenu);

        parkMenu.addChoice("Wall", Park.WALL, true);
        parkMenu.addChoice("Bridge", Park.BRIDGE, false);
        parkMenu.addChoice("None", Park.NONE, false);


        FtcMenu.walkMenuTree(modeMenu, this);
        runmode = modeMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentValue();
        alliance = allianceMenu.getCurrentChoiceObject();
        startposition = startPositionMenu.getCurrentChoiceObject();
        skystones = skystonesMenu.getCurrentChoiceObject();
        stones = stonesMenu.getCurrentChoiceObject();
        foundation = foundationMenu.getCurrentChoiceObject();
        park = parkMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(13, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(),
                runmode.toString());
        dashboard.displayPrintf(14, "Delay = %d msec", delay);
        dashboard.displayPrintf(15, "Alliance: %s (%s)",
                allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(16, "Start position: %s (%s)",
                startPositionMenu.getCurrentChoiceText(), startposition.toString());
        dashboard.displayPrintf(17, "Number of Skystones: %s (%s)",
                skystonesMenu.getCurrentChoiceText(), skystones.toString());
        dashboard.displayPrintf(18, "Number of Stones: %s (%s)",
                stonesMenu.getCurrentChoiceText(), stones.toString());
        dashboard.displayPrintf(19, "Move Foundation: %s (%s)",
                foundationMenu.getCurrentChoiceText(), foundation.toString());
        dashboard.displayPrintf(20, "Park: %s (%s)", parkMenu.getCurrentChoiceText(),
                park.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
