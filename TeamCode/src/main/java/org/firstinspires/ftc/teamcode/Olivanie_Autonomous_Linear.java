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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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
        YES,
        NO,
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
    Foundation foundation = Foundation.YES;
    Park park = Park.WALL;
    Auto_path path = Auto_path.PATH1;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    Olivanie_v2_Hardware robot = new Olivanie_v2_Hardware();

    // Skystone variables
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static double valMid = 0;
    private static double valLeft = 0;
    private static double valRight = 0;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

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
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR;
    static final double COUNTS_PER_DEGREE     = COUNTS_PER_INCH*0.173; // Found by testing

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
            if (startposition == StartPosition.BUILDING1) {
                if (skystones == Skystones.ZERO) {
                    if (stones == Stones.ZERO) {
                        if (foundation == Foundation.YES) {
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
                        if (foundation == Foundation.YES) {
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


        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveSetTargetPosition(0, 0, 0, 0);

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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new opencvSkystoneDetector.StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        // Sample while waiting for start
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Values", valLeft+"\n"+valMid+"\n"+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            if (valLeft < valMid && valLeft < valRight) {
                telemetry.addData("Skystone:", "Left");
                skystonePosition = SkystonePosition.LEFT;
            }
            else if (valMid < valLeft && valMid < valRight) {
                telemetry.addData("Skystone:", "Middle");
                skystonePosition = SkystonePosition.CENTER;
            }
            else {
                telemetry.addData("Skystone:", "Right");
                skystonePosition = SkystonePosition.RIGHT;
            }

            telemetry.update();
            sleep(100);
        }


        /**
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running");

        // For testing drive train motors and encoders
        if (DoTask("Raw motor test", runmode, false)) {
            robot.driveSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setPowerAll(.5);
            sleep(3000);
            robot.setPowerAll(0);
            sleep(1000);
            robot.setPowerAll(-.5);
            sleep(3000);
            robot.setPowerAll(0);
        }

        if (DoTask("Chassis motor test", runmode, false)) {
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setPowerAll(0.5);

            robot.rightDriveB.setTargetPosition(5000);
            while (robot.rightDriveB.isBusy()) {
                sleep(10);
            }

            robot.leftDriveB.setTargetPosition(5000);
            while (robot.leftDriveB.isBusy()) {
                sleep(10);
            }

            robot.rightDriveF.setTargetPosition(5000);
            while (robot.rightDriveF.isBusy()) {
                sleep(10);
            }

            robot.leftDriveF.setTargetPosition(5000);
            while (robot.leftDriveF.isBusy()) {
                sleep(10);
            }
        }


        if (DoTask("Spin test - clockwise", runmode, false))
            DriveRobotTurn(.4, 360 * 5);

        if (DoTask("Spin test - counter-clockwise", runmode, false))
            DriveRobotTurn(.4, -360 * 5);

        if (DoTask("Drive test (72 inches)", runmode, false))
            DriveRobotPosition(.4, 72, false);

        if (DoTask("Strafe test", runmode, false))
            DriveSideways(.4, 36);

        if (DoTask("Diagonal test", runmode, false))
            DriveDiagonal(.4, 36, true);

        // Pause the program for the selected delay period
        sleep(delay);

        // Init
        if (DoTask("Init", runmode, true)) {

        }

        if (DoTask("Drive", runmode, true)) {
            robot.collectorRev();
            sleep(500);
            robot.collectorOff();
            if (startposition == StartPosition.BUILDING1) {
                robot.openFoundation();
                DriveRobotPosition(.4, -32, false);
                robot.closeFoundation();
                sleep(800);
                DriveRobotPosition(.4, 24, false);
                if (alliance == Alliance.BLUE) {
                    DriveRobotArc(1, 12, -.7);
                    DriveRobotPosition(.3, 6, false);
                    DriveRobotArc(1, 12, -.7);
                } else if (alliance == Alliance.RED) {
                    DriveRobotArc(1, 12, .7);
                    DriveRobotPosition(.3, 6, false);
                    DriveRobotArc(1,9, .7);
                }
                if (park != Park.NONE) {
                    robot.openFoundation();
                    sleep(800);
                    DriveRobotPosition(.5, 3, false);
                    robot.closeFoundation();
                    sleep(800);
                    DriveRobotTime(2500, -.3);
                    robot.openFoundation();
                    sleep(800);
                    DriveRobotPosition(1, 10, false);
                    if ((alliance == Alliance.RED && park == Park.WALL) ||
                            (alliance == Alliance.BLUE && park == Park.BRIDGE)) {
                        DriveRobotTurn(.6, -30);
                        DriveRobotPosition(.5, 30, false);
                    }
                    else {
                        DriveRobotTurn(.6, 30);
                        DriveRobotPosition(.5, 30, false);
                    }
                    //DriveRobotPosition(.6, 30, false);
                }
            }
            else if (startposition == StartPosition.LOADING1) {
                if (foundation == Foundation.YES) {
                    if (skystones == Skystones.ZERO) {
                        DriveRobotPosition(.6, 24, false);
                        sleep(50);
                        if (alliance == Alliance.RED) {
                            DriveRobotTurn(.5, 45, false);
                        } else if (alliance == Alliance.BLUE) {
                            DriveRobotTurn(.5, -45, false);
                        }
                        robot.collectorOn();
                        sleep(50);
                        DriveRobotPosition(.5, 5, false);
                        DriveRobotPosition(.3, 5, false);
                        for (int i = 0; i < 4; i++) {
                            if (!isCollectorJammed()) {
                                sleep(50);
                            } else {
                                robot.collectorRev();
                                sleep(49);
                                robot.collectorOn();
                            }
                        }
                        DriveRobotPosition(.4, -12, false);
                        sleep(50);
                        if (alliance == Alliance.RED) {
                            DriveRobotTurn(.3, 45, false);
                        } else if (alliance == Alliance.BLUE) {
                            DriveRobotTurn(.3, -45, false);
                        }
                        DriveRobotPosition(.5, -36, false);
                        if (alliance == Alliance.RED) {
                            DriveRobotTurn(.4, 90, false);
                        } else if (alliance == Alliance.BLUE) {
                            DriveRobotTurn(.4, -90, false);
                        }
                        DriveRobotPosition(.4, -12, false);
                        robot.openFoundation();
                        sleep(1000);
                        DriveRobotPosition(.4, 16, false);
                        if (alliance == Alliance.BLUE) {
                            DriveRobotArc(1, 20, -.6);
                        } else if (alliance == Alliance.RED) {
                            DriveRobotArc(1, 20, .7);
                        }
                        robot.openFoundation();
                        sleep(800);
                        DriveRobotPosition(.5, 3, false);
                        robot.closeFoundation();
                        sleep(800);
                        DriveRobotTime(2500, -.3);
                        robot.openFoundation();
                        sleep(800);
                        DriveRobotPosition(1, 10, false);
                        DriveRobotTime(400, -1);
                        DriveRobotPosition(.6, 40, false);
                    }
                    else {

                    }
                }
                else {
                    DriveRobotPosition(.5, -6, false);
                }
            }
            else if (startposition == StartPosition.LOADING2) {
                DriveRobotPosition(.6, 12, false);
                sleep(50);
                DriveRobotTurn(.6, -90);
                if (alliance == Alliance.BLUE) {
                    if (skystonePosition == SkystonePosition.CENTER)
                        DriveRobotPosition(.3, -8, false);
                    else if (skystonePosition == SkystonePosition.LEFT)
                        DriveRobotPosition(.3, -16, false);
                }
                DriveSideways(.6, -10);
                sleep(50);
                robot.leftSideClaw.setPosition(robot.GRABBEDL);
                sleep(500);
                DriveSideways(.6, 10);
                sleep(50);
                DriveRobotPosition(.5, -48, false);
                robot.leftSideClaw.setPosition(robot.RELEASEDL);
                sleep(500);
                DriveRobotPosition(.5, 10, false);
            }
        }

    }

    /**
     * FUNCTIONS -----------------------------------------------------------------------------------
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
     * DriveRobotTime drives the robot the set number of inches at the given power level.
     * @param ms How long to drive
     * @param power Power level to set motors to, negative will drive the robot backwards
     */
    void DriveRobotTime(int ms, double power)
    {
        robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setPowerAll(-power);
        sleep(ms);
        robot.setPowerAll(0);
        robot.driveSetTargetPosition(0, 0, 0, 0);
    }


    /**
     * DriveRobotPosition drives the robot the set number of inches at the given power level.
     * @param inches How far to drive, can be negative
     * @param power Power level to set motors to
     */
    void DriveRobotPosition(double power, double inches, boolean smart_accel)
    {
        int state = 0; // 0 = NONE, 1 = ACCEL, 2 = DRIVE, 3 = DECEL
        double position = inches*COUNTS_PER_INCH;

        robot.driveSetRunToPosition();

        if (smart_accel && power > 0.25)
        {
            robot.setPowerAll(0.25); // Use abs() to make sure power is positive
            state = 1; // ACCEL
        }
        else {
            robot.setPowerAll(abs(power));
            //robot.setPowerAll(abs(power)); // Use abs() to make sure power is positive
        }

        int flOrigTarget = robot.leftDriveF.getTargetPosition();
        int frOrigTarget = robot.rightDriveF.getTargetPosition();
        int blOrigTarget = robot.leftDriveB.getTargetPosition();
        int brOrigTarget = robot.rightDriveB.getTargetPosition();
        robot.driveAddTargetPosition((int)position, (int)position, (int)position, (int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.driveAnyReachedTarget()) {
                int flDrive = robot.leftDriveF.getCurrentPosition();
                int frDrive = robot.rightDriveF.getCurrentPosition();
                int blDrive = robot.leftDriveB.getCurrentPosition();
                int brDrive = robot.rightDriveB.getCurrentPosition();
                dashboard.displayPrintf(3, "Front left encoder: %d", flDrive);
                dashboard.displayPrintf(4, "Front right encoder: %d", frDrive);
                dashboard.displayPrintf(5, "Back left encoder: %d", blDrive);
                dashboard.displayPrintf(6, "Back right encoder %d", brDrive);

                // State magic
                if (state == 1 &&
                        (abs(flDrive-flOrigTarget) > 2*COUNTS_PER_INCH ||
                         abs(frDrive-frOrigTarget) > 2*COUNTS_PER_INCH //||
                         //abs(blDrive-blOrigTarget) > 2*COUNTS_PER_INCH ||
                         /*abs(brDrive-brOrigTarget) > 2*COUNTS_PER_INCH */)) {
                    // We have gone 2 inches, go to full power
                    robot.setPowerAll(abs(power)); // Use abs() to make sure power is positive
                    state = 2;
                }
                else if (state == 2 &&
                        (abs(flDrive-flOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         abs(frDrive-frOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) //||
                         //abs(blDrive-blOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         /*abs(brDrive-brOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) */)) {
                    // Cut power by half to DECEL
                    robot.setPowerAll(abs(power)/2); // Use abs() to make sure power is positive
                    state = 3; // We are DECELing now
                }
                dashboard.displayPrintf(7, "State: %d (0=NONE,1=ACCEL,2=DRIVING,3=DECEL", state);
            }
            sleep(10);
        }

        robot.setPowerAll(0);
        // Clear used section of dashboard 
        dashboard.displayText(3, "");
        dashboard.displayText(4, "");
        dashboard.displayText(5, "");
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");
    }

    void DriveRobotTurn (double power, double degree, boolean smart_accel)
    {
        double position = -degree*COUNTS_PER_DEGREE;

        int state = 0; // 0 = NONE, 1 = ACCEL, 2 = DRIVE, 3 = DECEL

        robot.driveSetRunToPosition();

        if (smart_accel) {
            state = 1;
            robot.setPowerLeft(power * 0.5);
            robot.setPowerRight(-power * 0.5);
        }
        else
        {
            robot.setPowerLeft(power);
            robot.setPowerRight(-power);
        }

        int flOrigTarget = robot.leftDriveF.getTargetPosition();
        int frOrigTarget = robot.rightDriveF.getTargetPosition();
        int blOrigTarget = robot.leftDriveB.getTargetPosition();
        int brOrigTarget = robot.rightDriveB.getTargetPosition();
        robot.driveAddTargetPosition((int)position, -(int)position, (int)position, -(int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.driveAllAreBusy()) {
                int flDrive = robot.leftDriveF.getCurrentPosition();
                int frDrive = robot.rightDriveF.getCurrentPosition();
                int blDrive = robot.leftDriveB.getCurrentPosition();
                int brDrive = robot.rightDriveB.getCurrentPosition();
                dashboard.displayPrintf(3, "Front left encoder: %d", flDrive);
                dashboard.displayPrintf(4, "Front right encoder: %d", frDrive);
                dashboard.displayPrintf(5, "Back left encoder: %d", blDrive);
                dashboard.displayPrintf(6, "Back right encoder %d", brDrive);

                // State magic
                if (state == 1 &&
                        (abs(flDrive-flOrigTarget) > COUNTS_PER_DEGREE*10 ||
                         abs(frDrive-frOrigTarget) > COUNTS_PER_DEGREE*10 ||
                         abs(blDrive-blOrigTarget) > COUNTS_PER_DEGREE*10 ||
                         abs(brDrive-brOrigTarget) > COUNTS_PER_DEGREE*10 )) {
                    // We have rotated 10 degrees, go to full power
                    robot.setPowerAll(abs(power)); // Use abs() to make sure power is positive
                    state = 2;
                }
                else if (state == 2 &&
                        (abs(flDrive-flOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(frDrive-frOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(blDrive-blOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(brDrive-brOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) )) {
                    // We are within 10 degrees of our destination, cut power by half to DECEL
                    robot.setPowerAll(abs(power)/2); // Use abs() to make sure power is positive
                    state = 3; // We are DECELing now
                }
                dashboard.displayPrintf(7, "State: %d (0=NONE,1=ACCEL,2=DRIVING,3=DECEL", state);
            }
            sleep(10);
        }
        robot.setPowerAll(0);
        // Clear used section of dashboard 
        dashboard.displayText(3, "");
        dashboard.displayText(4, "");
        dashboard.displayText(5, "");
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");
    }

    /** For compatibility */
    void DriveRobotTurn (double power, double degree)
    {
        DriveRobotTurn(power, -degree, false);
    }


    void DriveRobotArc(double power, double inches, double difference)
    {
        double position = -inches*COUNTS_PER_INCH;
        difference = Range.clip(difference, -1, 1);
//power 1, inches -48, difference -.5
        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (difference > 0) {
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (difference > 0 && inches > 0) {
            robot.setPowerLeft(abs(power));
            robot.setPowerRight(abs(power * difference));
        }
        else if (difference > 0 && inches < 0) {
            robot.setPowerLeft(abs(power));
            robot.setPowerRight(-abs(power * difference));
        }
        else if (difference < 0 && inches > 0) {
            robot.setPowerLeft(abs(power * difference));
            robot.setPowerRight(abs(power));
        }
        else if (difference < 0 && inches < 0) {
            robot.setPowerLeft(-abs(power * difference));
            robot.setPowerRight(abs(power));
        }

        if (difference > 0) {
            robot.leftDriveF.setTargetPosition((int)position);
            robot.leftDriveB.setTargetPosition((int)position);
        }
        else {
            robot.rightDriveF.setTargetPosition((int)position);
            robot.rightDriveB.setTargetPosition((int)position);
        }
        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            if (difference > 0) {
                while ((Math.abs(robot.leftDriveF.getCurrentPosition() - robot.leftDriveF.getTargetPosition()) > 50.0)
                        && (Math.abs(robot.leftDriveB.getCurrentPosition() - robot.leftDriveB.getTargetPosition()) > 50.0)) {
                    int flDrive = robot.leftDriveF.getCurrentPosition();
                    dashboard.displayPrintf(3, "Front left encoder: %d", flDrive);
                }
            }
            else {
                while ((Math.abs(robot.rightDriveF.getCurrentPosition() - robot.rightDriveB.getTargetPosition()) > 50.0)
                        && (Math.abs(robot.rightDriveB.getCurrentPosition() - robot.rightDriveB.getTargetPosition()) > 50.0)) {
                    int frDrive = robot.rightDriveF.getCurrentPosition();
                    dashboard.displayPrintf(3, "Front right encoder: %d", frDrive);
                    dashboard.displayPrintf(4, "count %d", i);
                }
            }
            sleep(10);
        }

        robot.setPowerAll(0);
    }

    void DriveArcTurn (double powerL, double powerR, double degrees) {
        double radians = 0;
        double l = robot.getLeftWheelEncoder();
        double r = robot.getRightWheelEncoder();
        robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setPowerLeft(powerL);
        robot.setPowerRight(powerR);
        while (Math.abs(Math.toDegrees(radians)) < Math.abs(degrees)) {
            radians =  -((robot.getLeftWheelEncoder() - l) - (robot.getRightWheelEncoder() - r))
                    / robot.DISTANCE_BETWEEN_WHEELS;
        }
        robot.setPowerAll(0);
    }

    void DriveDiagonal (double power, double inches, boolean right) {
        double position = Math.hypot(inches, inches) * COUNTS_PER_INCH;
        if ((right && power > 0) || (!right && power < 0)) {
            robot.leftDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveF.setPower(power);
            robot.rightDriveB.setPower(power);
            robot.leftDriveF.setTargetPosition((int) position);
            robot.rightDriveB.setTargetPosition((int) position);
        }
        else {
            robot.rightDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDriveF.setPower(power);
            robot.leftDriveB.setPower(power);
            robot.rightDriveF.setTargetPosition((int) position);
            robot.leftDriveB.setTargetPosition((int) position);
        }
        for (int i = 0; i < 5; i++) {
            if ((right && power > 0) || (!right && power < 0)) {
                while ((Math.abs(robot.leftDriveF.getCurrentPosition()
                        - robot.leftDriveF.getTargetPosition()) > 50.0) &&
                        (Math.abs(robot.rightDriveB.getCurrentPosition()
                                - robot.rightDriveB.getCurrentPosition()) > 50.0)) {
                    int flDrive = robot.leftDriveF.getCurrentPosition();
                    dashboard.displayPrintf(3, "Front left encoder: %d", flDrive);
                }
                while ((Math.abs(robot.rightDriveF.getCurrentPosition()
                        - robot.rightDriveF.getTargetPosition()) > 50.0) &&
                        (Math.abs(robot.leftDriveB.getCurrentPosition()
                                - robot.leftDriveB.getCurrentPosition()) > 50.0)) {
                    int frDrive = robot.rightDriveF.getCurrentPosition();
                    dashboard.displayPrintf(3, "Front right encoder: %d", frDrive);
                }
            }
            sleep(10);
        }
        robot.setPowerAll(0);
    }

    void DriveSideways (double power, double inches) {
        if (inches < 0) {
            inches *= -1;
            power *= -1;
        }
        double position = inches*COUNTS_PER_INCH*1.1;

        robot.driveSetRunToPosition();
        if (power > 0) {
            robot.leftDriveF.setPower(power);
            robot.rightDriveF.setPower(power);
            robot.leftDriveB.setPower(power);
            robot.rightDriveB.setPower(power);
            robot.driveAddTargetPosition((int)position, (int)-position, (int)-position, (int)position);
        }
        else {
            robot.leftDriveF.setPower(-power);
            robot.rightDriveF.setPower(-power);
            robot.leftDriveB.setPower(-power);
            robot.rightDriveB.setPower(-power);
            robot.driveAddTargetPosition((int)-position, (int)position, (int)position, (int)-position);
        }
//left
        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.driveAnyReachedTarget()) {
                int flDrive = robot.leftDriveF.getCurrentPosition();
                int frDrive = robot.rightDriveF.getCurrentPosition();
                int blDrive = robot.leftDriveB.getCurrentPosition();
                int brDrive = robot.rightDriveB.getCurrentPosition();
                dashboard.displayPrintf(3, "Front left encoder: %d", flDrive);
                dashboard.displayPrintf(4, "Front right encoder: %d", frDrive);
                dashboard.displayPrintf(5, "Back left encoder: %d", blDrive);
                dashboard.displayPrintf(6, "Back right encoder %d", brDrive);
            }
            sleep(10);
        }
        robot.setPowerAll(0);
        // Clear used section of dashboard
        dashboard.displayText(3, "");
        dashboard.displayText(4, "");
        dashboard.displayText(5, "");
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");
    }

    boolean isCollectorJammed () {
        if (robot.leftCollector.getVelocity() < 10)
            return true;
        else
            return false;
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

        private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;
        private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages = opencvSkystoneDetector.StageSwitchingPipeline.Stage.values();

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
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid;//gets value at circle
            double[] pixLeft;//gets value at circle
            double[] pixRight;//gets value at circle
            double midSum = 0;
            double leftSum = 0;
            double rightSum = 0;
            for (int i = - 10; i < 10; i++) {
                for (int j = -10; j < 10; j++) {
                    pixMid = thresholdMat.get((int)(input.rows()* midPos[1]) + i, (int)(input.cols()* midPos[0]) + j);
                    pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]) + i, (int)(input.cols()* leftPos[0]) + j);
                    pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]) + i, (int)(input.cols()* rightPos[0]) + j);
                    midSum += pixMid[0];
                    leftSum += pixLeft[0];
                    rightSum += pixRight[0];
                }
            }

            valMid = midSum / 400;
            valLeft = leftSum / 400;
            valRight = rightSum / 400;

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,10, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,10, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,10, new Scalar( 255, 0, 0 ),1 );//draws circle

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


    private void doMenus() {
        FtcChoiceMenu<RunMode> modeMenu = new FtcChoiceMenu<>("Run Mode", null, this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay:", modeMenu, this, 0, 20000, 1000, 0, "%.0f msec");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", delayMenu, this);
        FtcChoiceMenu<StartPosition> startPositionMenu = new FtcChoiceMenu<>("Start Position:", allianceMenu, this);
        FtcChoiceMenu<Skystones> skystonesMenu = new FtcChoiceMenu<>("Number of Skystones:", startPositionMenu, this);
        FtcChoiceMenu<Stones> stonesMenu = new FtcChoiceMenu<>("Number of Stones:", skystonesMenu, this);
        FtcChoiceMenu<Foundation> foundationMenu = new FtcChoiceMenu<>("Move the Foundation:", stonesMenu, this);
        FtcChoiceMenu<Park> parkMenu = new FtcChoiceMenu<>("Park :", foundationMenu, this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, delayMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, delayMenu);

        delayMenu.setChildMenu(allianceMenu);

        allianceMenu.addChoice("Red", Alliance.RED, true, startPositionMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, startPositionMenu);

        startPositionMenu.addChoice("Loading Zone Far", StartPosition.LOADING1, false, skystonesMenu);
        startPositionMenu.addChoice("Loading Zone Near", StartPosition.LOADING2, true, skystonesMenu);
        startPositionMenu.addChoice("Building Zone Far", StartPosition.BUILDING1, false, skystonesMenu);
        startPositionMenu.addChoice("Building Zone Near", StartPosition.BUILDING2, false, skystonesMenu);

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

        foundationMenu.addChoice("Yes", Foundation.YES, true, parkMenu);
        foundationMenu.addChoice("No", Foundation.NO, true, parkMenu);

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

        dashboard.displayPrintf(13, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(14, "Delay = %d msec", delay);
        dashboard.displayPrintf(15, "Alliance: %s (%s)", allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(16, "Start position: %s (%s)", startPositionMenu.getCurrentChoiceText(), startposition.toString());
        dashboard.displayPrintf(17, "Number of Skystones: %s (%s)", skystonesMenu.getCurrentChoiceText(), skystones.toString());
        dashboard.displayPrintf(18, "Number of Stones: %s (%s)", stonesMenu.getCurrentChoiceText(), stones.toString());
        dashboard.displayPrintf(19, "Move Foundation: %s (%s)", foundationMenu.getCurrentChoiceText(), foundation.toString());
        dashboard.displayPrintf(20, "Park: %s (%s)", parkMenu.getCurrentChoiceText(), park.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
