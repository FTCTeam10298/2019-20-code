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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

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
    Stones stones = Stones.ZERO;
    Foundation foundation = Foundation.YES;
    Park park = Park.WALL;
    Auto_path path = Auto_path.PATH1;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    Olivanie_Hardware robot = new Olivanie_Hardware();

    static final double COUNTS_PER_MOTOR_REV  = 28.0;      // Rev HD Hex v2.1 Motor encoder
    static final double GEARBOX_RATIO         = 20.0;      // 40 for 40:1, 20 for 20:1
    static final double DRIVE_GEAR_REDUCTION  = 1; // This is > 1.0 if geared for torque
    static final double WHEEL_DIAMETER_INCHES = 3.937007874015748; // For figuring circumference
    static final double DRIVETRAIN_ERROR      = 1.04;      // Error determined from testing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR;
    static final double COUNTS_PER_DEGREE     = (COUNTS_PER_INCH*0.20672)+0.003703704; // Was 0.20672; // Found by testing

    @Override
    public void runOpMode() {
        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap);

        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry);



        // Run though the menu ---------------------------------------------------------------------
        doMenus();

//        if (startposition == StartPosition.SILVER && crater == Crater.FAR) {
//            dashboard.displayPrintf(1, "You picked the wrong option, Stephanie!");
//            crater = Crater.NEAR;
//            errors += 1;
//        }

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

        waitForStart();


        /**
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running");

        // For testing drive train motors and encoders
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
            DriveRobotTurn(.2, 360 * 5);

        if (DoTask("Spin test - counter-clockwise", runmode, false))
            DriveRobotTurn(.2, -360 * 5);

        if (DoTask("Drive test (72 inches)", runmode, false))
            DriveRobotPosition(.2, 72, false);

        // Pause the program for the selected delay period
        sleep(delay);

        // Init
        if (DoTask("Init", runmode, true)) {

        }

        if (DoTask("Drive", runmode, true)) {
            robot.openFoundation();
            DriveRobotPosition(.3, -32, false);
            robot.closeFoundation();
            sleep(1000);
            DriveRobotPosition(.4, 16, false);
            if (path == Auto_path.PATH1) {
                DriveRobotArc(1, 30, -.5);
            }
            else if (path == Auto_path.PATH2) {
                DriveRobotArc(1, 30, .5);
            }
            robot.openFoundation();
            sleep(1000);
            DriveRobotPosition(.5, 3, false);
            robot.closeFoundation();
            sleep(1000);
            DriveRobotTime(3000, -.3);
            DriveRobotPosition(.5, 40, false);
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
        double position = -inches*COUNTS_PER_INCH;

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
        double position = degree*COUNTS_PER_DEGREE;

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
        foundationMenu.addChoice("1", Foundation.NO, true, parkMenu);

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
