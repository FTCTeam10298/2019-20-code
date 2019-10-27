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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

@Autonomous(name="Olivanie Autonomous Odometry", group ="Olivanie")
public class Olivanie_Autonomous_Odometry extends OpMode implements FtcMenu.MenuButtons {

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
        PATH2
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
    Robosition position = new Robosition();
    RoboPoint roboPoint = new RoboPoint();

    static final double PROPORTIONAL_TERM = 1;
    static final double INTEGRAL_TERM = 0;
    static final double DERIVATIVE_TERM = 0;

    static final double COUNTS_PER_MOTOR_REV  = 28.0;      // Rev HD Hex v2.1 Motor encoder
    static final double GEARBOX_RATIO         = 20.0;      // 40 for 40:1, 20 for 20:1
    static final double DRIVE_GEAR_REDUCTION  = 24.0/15.0; // This is > 1.0 if geared for torque
    static final double WHEEL_DIAMETER_INCHES = 3.937007874015748; // For figuring circumference
    static final double DRIVETRAIN_ERROR      = 1.04;      // Error determined from testing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR;

    int errors = 0;

    // code to run once after driver hits init
    @Override
    public void init() {
        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap);

        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry);

        // Run though the menu ---------------------------------------------------------------------
        doMenus();

        if (alliance == Alliance.RED) {
            if (startposition == StartPosition.BUILDING1) {
                if (skystones == Skystones.ZERO) {
                    if (stones == Stones.ZERO) {
                        if (foundation == Foundation.YES) {
                            if (park == Park.WALL) {
                                path = Auto_path.PATH1;
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
                        }
                    }
                }
            }
        }

        dashboard.displayPrintf(0, "Status: Ready to start");
        if (errors > 0)
            dashboard.displayPrintf(2, "!!! %d errors!", errors);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        /**
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running");

        driveToPoint(1, new WayPoint(12, 24, 0, 0));
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
        @Override
        public void loop () {

        }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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
    boolean DoTask(String taskname, RunMode debug, boolean default_value)
    {
        dashboard.displayPrintf(0, taskname);
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard.displayPrintf(1, "Press A to run, B to skip");
                if (gamepad1.a) {
                    dashboard.displayPrintf(1, "Run");
                    return true;
                }
                if (gamepad1.b) {
                    dashboard.displayPrintf(1, "Skip");
                    //sleep
                    return false;
                }
        }
        return default_value;
    }

    /**
     * Drives straight using PID
     * @deprecated Use driveToPoint instead, adds turning
     * @param power Power
     * @param inches Inches
     */
    void PIDDriveStraight (double power, double inches) {

        double output;
        double desiredDistance = inches;

        if (power > 0 && desiredDistance < 0) {
            power *= 1;
        }
        if (power < 0 && desiredDistance > 0) {
            desiredDistance *= -1;
        }

        double desiredX = roboPoint.getX() + (desiredDistance
                * Math.sin(Math.toRadians(roboPoint.getAngle())));
        double desiredY = roboPoint.getY() + (desiredDistance
                * Math.cos(Math.toRadians(roboPoint.getAngle())));
        double counter = 0;
        double previousTime = getRuntime();
        double previousError = roboPoint.getDistanceAway(desiredX, desiredY);
        double I = 0;

        WayPoint wayPoint = new WayPoint(desiredX, desiredY, roboPoint.getAngle(), 0);

        while (!roboPoint.isPointReached(wayPoint)) {

            double leftEncoder = ((double) robot.leftDriveF.getCurrentPosition()
                    + (double) robot.leftDriveB.getCurrentPosition()) / 2;
            double rightEncoder = ((double) robot.rightDriveF.getCurrentPosition()
                    + (double) robot.rightDriveB.getCurrentPosition()) / 2;
            double currentTime = getRuntime();
            double currentError = roboPoint.getDistanceAway(desiredX, desiredY);

            I += currentError * (currentTime - previousTime);


            // PID
            if (power > 0)
                output = Range.clip(PROPORTIONAL_TERM * currentError, -1, power);
            else
                output = Range.clip(PROPORTIONAL_TERM * currentError, power, 1);
            output += INTEGRAL_TERM * I;
            output += DERIVATIVE_TERM * ((currentError - previousError)
                    / (currentTime - previousTime));

            // Set Power
            if (Math.abs(power * counter) > Math.abs(output))
                robot.setPowerAll(output);
            else {
                if (power > 0)
                    robot.setPowerAll(Range.clip(power * counter, -1, power));
                else
                    robot.setPowerAll(Range.clip(power * counter, power, 1));
            }

            // Update values
            counter += 0.01;
            roboPoint.updatePosition((((double)robot.leftDriveF.getCurrentPosition()
                    + (double)robot.leftDriveB.getCurrentPosition())/2)
                    - leftEncoder, (((double)robot.rightDriveF.getCurrentPosition()
                    + (double)robot.rightDriveB.getCurrentPosition())/2)
                    - rightEncoder);
            previousError = currentError;
            previousTime = currentTime;
        }
    }

    void PIDTurn (double power, double degrees) {

        double speed = Math.abs(power);
        double output;
        double desiredAngle = degrees + roboPoint.getAngle();

        if (desiredAngle > 180)
            desiredAngle -= 360;
        else if (desiredAngle < -180)
            desiredAngle += 360;

        WayPoint wayPoint = new WayPoint(roboPoint.getX(), roboPoint.getY(), desiredAngle,0);

        double counter = 0;
        double previousTime = getRuntime();
        double previousError = roboPoint.getAngleAway(wayPoint);
        double I = 0;
        boolean right = roboPoint.getAngleAway(wayPoint) > 0;

        while (!roboPoint.isAngleReached(wayPoint)) {
            double leftEncoder = robot.getLeftWheelEncoder();
            double rightEncoder = robot.getRightWheelEncoder();
            double currentTime = getRuntime();
            double currentError = roboPoint.getAngleAway(wayPoint);

            I += currentError * (currentTime - previousTime);

            // PID
            output = Range.clip(PROPORTIONAL_TERM * currentError, -1, Math.abs(speed));
            output += INTEGRAL_TERM * I;
            output += DERIVATIVE_TERM * ((currentError - previousError)
                    / (currentTime - previousTime));

            // Set Power
            if (Math.abs(speed * counter) > Math.abs(output)) {
                if (right) {
                    robot.setPowerLeft(Range.clip(output, 1, speed));
                    robot.setPowerRight(-Range.clip(output, 1, speed));
                }
                else {
                    robot.setPowerLeft(-Range.clip(output, 1, speed));
                    robot.setPowerRight(Range.clip(output, 1, speed));
                }
            }
            else {
                if (right) {
                    robot.setPowerLeft(Range.clip(Math.abs(speed * counter), 0,
                            speed));
                    robot.setPowerRight(-Range.clip(Math.abs(speed * counter), -speed,
                            0));
                }
                else {
                    robot.setPowerLeft(-Range.clip(Math.abs(speed * counter), -speed,
                            0));
                    robot.setPowerRight(Range.clip(Math.abs(speed * counter), 0,
                            speed));
                }
            }

            counter += 0.01;
            roboPoint.updatePosition(robot.getLeftWheelEncoder() - leftEncoder,
                    robot.getRightWheelEncoder() - rightEncoder);
            previousError = currentError;
            previousTime = currentTime;
        }
    }

    void driveToPoint (double power, Point point) {

        double speed = Math.abs(power);
        double drive;
        double outputPIDStraight;
        double outputPIDTurn;
        double turn;
        double counter = 0;
        double previousTime = getRuntime();
        double previousError = roboPoint.getDistanceAway(point);
        double previousHeadingError = roboPoint.getHeadingError(point);
        double I = 0;

        while (!roboPoint.isPointReached(point)) {

            double currentTime = getRuntime();
            double currentError = roboPoint.getDistanceAway(point);
            double currentHeadingError = roboPoint.getHeadingError(point);
            double leftEncoder = robot.getLeftWheelEncoder();
            double rightEncoder = robot.getRightWheelEncoder();

            WayPoint wayPoint = new WayPoint(currentHeadingError);

            int driveDirection;
            if (Math.abs(roboPoint.getAngleAway(wayPoint)) > 90)
                driveDirection = -1;
            else
                driveDirection = 1;

            int turnDirection;
            if (roboPoint.getAngleAway(wayPoint) > 0)
                turnDirection = 1;
            else
                turnDirection = -1;

            I += currentError * (currentTime - previousTime);

            // PID
            outputPIDStraight = Range.clip(PROPORTIONAL_TERM * currentError, -1, speed);
            outputPIDStraight += INTEGRAL_TERM * I;
            outputPIDStraight += DERIVATIVE_TERM * ((currentError - previousError)
                    / (currentTime - previousTime));
            outputPIDTurn = Range.clip(PROPORTIONAL_TERM * currentHeadingError, -1, speed);
            outputPIDTurn += INTEGRAL_TERM * I;
            outputPIDTurn += DERIVATIVE_TERM * ((currentHeadingError - previousHeadingError)
                    / (currentTime - previousTime));

            // Set Power
            if (Math.abs(speed * counter) > Math.abs(outputPIDStraight)) {
                drive = outputPIDStraight;
                turn = outputPIDTurn;
            }
            else {
                drive = Range.clip(speed * counter, -1, speed);
                turn = Range.clip(speed * counter, -1, speed);
            }

            drive *= driveDirection;
            turn *= turnDirection;

            robot.setPowerRight(Range.clip(drive + turn, -1, 1));
            robot.setPowerLeft(Range.clip(drive - turn, -1, 1));

            // Update values
            counter += 0.01;
            roboPoint.updatePosition(robot.getLeftWheelEncoder() - leftEncoder,
                    robot.getRightWheelEncoder() - rightEncoder);
            previousError = currentError;
            previousTime = currentTime;
        }
        PIDTurn(power, point.getAngle() - roboPoint.getAngle()); //Eventually add to above w/ PID
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
            robot.setPowerAll(Math.abs(power)); // Use abs() to make sure power is positive
        }

        int flOrigTarget = robot.leftDriveF.getTargetPosition();
        int frOrigTarget = robot.rightDriveF.getTargetPosition();
        int blOrigTarget = robot.leftDriveB.getTargetPosition();
        int brOrigTarget = robot.rightDriveB.getTargetPosition();
        robot.driveAddTargetPosition((int)position, (int)position, (int)position, (int)position);

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
                        (Math.abs(flDrive-flOrigTarget) > 2*COUNTS_PER_INCH ||
                                Math.abs(frDrive-frOrigTarget) > 2*COUNTS_PER_INCH ||
                                Math.abs(blDrive-blOrigTarget) > 2*COUNTS_PER_INCH ||
                                Math.abs(brDrive-brOrigTarget) > 2*COUNTS_PER_INCH )) {
                    // We have gone 2 inches, go to full power
                    robot.setPowerAll(Math.abs(power)); // Use abs() to make sure power is positive
                    state = 2;
                }
                else if (state == 2 &&
                        (Math.abs(flDrive-flOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2) ||
                                Math.abs(frDrive-frOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2) ||
                                Math.abs(blDrive-blOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2) ||
                                Math.abs(brDrive-brOrigTarget) > COUNTS_PER_INCH*(Math.abs(inches)-2) )) {
                    // Cut power by half to DECEL
                    robot.setPowerAll(Math.abs(power)/2); // Use abs() to make sure power is positive
                    state = 3; // We are DECELing now
                }
                dashboard.displayPrintf(7, "State: %d (0=NONE,1=ACCEL,2=DRIVING,3=DECEL", state);
            }
            //sleep(10);
        }

        robot.setPowerAll(0);
        // Clear used section of dashboard
        dashboard.displayText(3, "");
        dashboard.displayText(4, "");
        dashboard.displayText(5, "");
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");
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


        FtcMenu.walkMenuTree(modeMenu);
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