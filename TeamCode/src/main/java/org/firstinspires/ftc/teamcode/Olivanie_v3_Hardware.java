package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 * This class is extended by RoboMovement, which has all of the autonomous drive functions as well.
 */

public class Olivanie_v3_Hardware
{

    /* Public OpMode members. */
    public DcMotorEx leftDriveF = null;
    public DcMotorEx rightDriveF = null;
    public DcMotorEx leftDriveB = null;
    public DcMotorEx rightDriveB = null;
    public DcMotor leftCollector = null;
    public DcMotor rightCollector = null;
    public DcMotor tape = null;
    public DcMotor lift = null;
    public Servo claw = null;
    public Servo foundation = null;
    public Servo left4Bar = null;
    public Servo right4Bar = null;
    public Servo leftSideClaw = null;
    public Servo rightSideClaw = null;
    public Servo leftFinger = null;
    public Servo rightFinger = null;
    public Servo kniod = null;

    public static final double HELDL = .3;
    public static final double DROPPEDL = .67;
    public static final double HELDR = .7;
    public static final double DROPPEDR = .33;
    public static final double FINGERGRABBEDR = 0.8;
    public static final double FINGERRELEASER = 0.2;
    public static final double FINGERGRABBEDL = 0.2;
    public static final double FINGERRELEASEL = 0.8;
    public static final int BLOCK01 = -1000;
    public static final int BLOCK02 = -2300;
    public static final int BLOCK03 = -3400;
    public static final int BLOCK04 = -1000;
    public static final int BLOCK05 = -2100;
    public static final int BLOCK06 = -3200;
    public static final int BLOCK07 = -4300;
    public static final int BLOCK08 = -5400;
    public static final int BLOCK09 = -6500;
    public static final int BLOCK10 = -7600;
    public static final int [] LIFTPOSITION = {0, BLOCK01, BLOCK02, BLOCK03, BLOCK04, BLOCK05,
            BLOCK06, BLOCK07, BLOCK08, BLOCK09, BLOCK10};

    double deltaL = 0;
    double deltaC = 0;
    double deltaR = 0;
    double previousL = 0;
    double previousC = 0;
    double previousR = 0;

    RevBulkData bulkData;
    ExpansionHubMotor lOWheel, rOWheel, cOWheel, motor3;
    ExpansionHubEx expansionHub;

    Global_Robot globalRobot = new Global_Robot(0, 0, 180);

    /* Local OpMode members. */
    HardwareMap hwMap              = null;

    /* Constructor */
    public Olivanie_v3_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        leftDriveF = (DcMotorEx) hwMap.dcMotor.get("left drive f");
        rightDriveF = (DcMotorEx) hwMap.dcMotor.get("right drive f");
        leftDriveB = (DcMotorEx) hwMap.dcMotor.get("left drive b");
        rightDriveB = (DcMotorEx) hwMap.dcMotor.get("right drive b");
        leftCollector = hwMap.dcMotor.get("left collector");
        rightCollector = hwMap.dcMotor.get("right collector");
        tape = hwMap.dcMotor.get("tape");
        lift = hwMap.dcMotor.get("lift");

        // Define and initialize servos
        claw = hwMap.servo.get("claw");
        foundation = hwMap.servo.get("foundation");
        left4Bar = hwMap.servo.get("left 4 bar");
        right4Bar = hwMap.servo.get("right 4 bar");
        leftSideClaw = hwMap.servo.get("left side claw");
        rightSideClaw = hwMap.servo.get("right side claw");
        leftFinger = hwMap.servo.get("left finger");
        rightFinger = hwMap.servo.get("right finger");
        kniod = hwMap.servo.get("kniod");

        // Set direction for all motors
        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);

        leftCollector.setDirection(DcMotor.Direction.FORWARD);
        rightCollector.setDirection(DcMotor.Direction.REVERSE);
        tape.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDriveF.setPower(0);
        rightDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveB.setPower(0);
        leftCollector.setPower(0);
        tape.setPower(0);
        lift.setPower(0);

        // Set all servos to default positions
        openClaw();
        openFoundation();
        close4Bar();
        kniod.setPosition(.5);
        leftSideClaw.setPosition(0);
        rightSideClaw.setPosition(1);
        leftFinger.setPosition(FINGERGRABBEDL);
        rightFinger.setPosition(FINGERGRABBEDR);

        // Set motors to use brake mode
        leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tape.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use coast on these
        leftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Stop and reset the encoders on the necessary motors
        tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set drive motors to run with encoders
        leftDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Odometry encoders on these
        leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tape.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Lift uses encoder
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Bulk Data
        expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 5");

        lOWheel = (ExpansionHubMotor) hwMap.dcMotor.get("left collector");
        cOWheel = (ExpansionHubMotor) hwMap.dcMotor.get("tape");
        rOWheel = (ExpansionHubMotor) hwMap.dcMotor.get("left drive b");
    }

    /**
     * FUNCTIONS
     */

    /**
     * driveSetPower sets all of the drive train motors to the specified power levels.
     * @param powerLF Power level to set front left motor to
     * @param powerRF Power level to set front right motor to
     * @param powerLB Power level to set back left motor to
     * @param powerRB Power level to set back right motor to
     */
    public void driveSetPower (double powerLF, double powerLB, double powerRF, double powerRB) {
        leftDriveF.setPower(powerLF);
        rightDriveF.setPower(powerRF);
        leftDriveB.setPower(powerLB);
        rightDriveB.setPower(powerRB);
    }

    /**
     * Sets the location of the global robot in the field.
     * @param x The x coordinate in inches.
     * @param y The y coordinate in inches.
     * @param angle The angle in radians.
     */
    public void setGlobalRobot (double x, double y, double angle) {
        setX(x);
        setY(y);
        setAngle(angle);
    }

    /**
     * Sets the x coordinate of the global robot.
     * @param x The x coordinate in inches.
     */
    public void setX (double x) {
        globalRobot.setX(x);
    }

    /**
     * Sets the y coordinate of the global robot.
     * @param y The y coordinate in inches.
     */
    public void setY (double y) {
        globalRobot.setY(y);
    }

    /**
     * Sets the angle of the global robot.
     * @param angle The angle in radians.
     */
    public void setAngle (double angle) {
        globalRobot.setAngle(angle);
    }

    /**
     * Gets the x coordinate of the robot.
     * @return The x coordinate in inches.
     */
    public double getX() {
        return globalRobot.getX();
    }

    /**
     * Gets the y coordinate of the robot.
     * @return The y coordinate in inches.
     */
    public double getY() {
        return globalRobot.getY();
    }

    /**
     * Gets the global angle in radians.
     * @return The angle in question.
     */
    public double getWorldAngle_rad () {
        return globalRobot.getAngle();
    }

    /**
     * Sets the position of the foundation moving servo to grab the foundation.
     */
    public void closeFoundation() {
        foundation.setPosition(1);
    }

    /**
     * Sets the position of the foundation moving servo to release the foundation.
     */
    public void openFoundation() {
        foundation.setPosition(0.35);
    }

    /**
     * Run the collector motors at a given power.
     * @param power The power to run the collector.
     */
    public void runCollector (double power) {
        leftCollector.setPower(power);
        rightCollector.setPower(power);
    }

    /**
     * Turns on the collector.
     */
    public void collectorOn () {
        runCollector(-1);
    }

    /**
     * Turns off the collector.
     */
    public void collectorOff () {
        runCollector(0);
    }

    /**
     * Reverses the collector.
     */
    public void  collectorRev () {
        runCollector(1);
    }

    /**
     * Sets the position of the 4 bar.
     * @param position The desired position for the left servo. The right one then mirrors it.
     */
    public void set4Bar (double position) {
        left4Bar.setPosition(position);
        right4Bar.setPosition(1 - position);
    }

    /**
     * Swings the 4 bar in to grab a stone
     */
    public void close4Bar () {
        set4Bar(1);
    }

    /**
     * Swings the 4 bar out to place a stone for the first 3 stones.
     */
    public void place4BarLow() {
        set4Bar(.05);
    }

    /**
     * Swings the 4 bar out to place a stone for stone 4 and above.
     */
    public void place4BarHigh() {
        set4Bar(.45);
    }

    /**
     * Opens the claw to place a stone.
     */
    public void openClaw () {
        claw.setPosition(1);
    }

    /**
     * Closes the claw to pick up a stone.
     */
    public void closeClaw() {
        claw.setPosition(.35);
    }

    /**
     * driveSetMode sets all of the drive train motors to the specified mode.
     * @param runmode RunMode to set motors to
     */
    void driveSetMode(DcMotor.RunMode runmode)
    {
        leftDriveF.setMode(runmode);
        rightDriveF.setMode(runmode);
        leftDriveB.setMode(runmode);
        rightDriveB.setMode(runmode);
    }

    /**
     * Sets the speed of the four drive motors given desired speeds in the robot's x, y, and angle.
     * @param vX Robot speed in the x (sideways) direction.
     * @param vY Robot speed in the y (forwards) direction.
     * @param vA Robot speed in the angle (turning) direction.
     * @param minPower Minimum speed allowed for the average of the four motors.
     * @param maxPower Maximum speed allowed for the fasted of the four motors.
     */
    public void setSpeedAll (double vX, double vY, double vA, double minPower, double maxPower) {
        // Calculate theoretical values for motor powers using transformation matrix
        double fl = vY + vX - vA;
        double bl = vY - vX - vA;
        double br = vY - vX + vA;
        double fr = vY + vX + vA;
        // Find the largest magnitude of power and the average magnitude of power to scale down to
        // maxpower and up to minpower
        double max = Math.abs(fl);
        max = Math.max(max, Math.abs(bl));
        max = Math.max(max, Math.abs(br));
        max = Math.max(max, Math.abs(fr));
        double ave = (Math.abs(fl) + Math.abs(bl) + Math.abs(br) + Math.abs(fr)) / 4;
        if (max > maxPower) {
            fl *= maxPower;
            bl *= maxPower;
            br *= maxPower;
            fr *= maxPower;
            fl /= max + 1E-6;
            bl /= max + 1E-6;
            br /= max + 1E-6;
            fr /= max + 1E-6;
        }
        else if (ave < minPower) {
            fl *= minPower;
            bl *= minPower;
            br *= minPower;
            fr *= minPower;
            fl /= max + 1E-6;
            bl /= max + 1E-6;
            br /= max + 1E-6;
            fr /= max + 1E-6;
        }
        // Recalculate max and scale down to 1
        max = Math.abs(fl);
        max = Math.max(max, Math.abs(bl));
        max = Math.max(max, Math.abs(br));
        max = Math.max(max, Math.abs(fr));
        if (max < 1)
            max = 1;
        fl /= max + 1E-6;
        bl /= max + 1E-6;
        br /= max + 1E-6;
        fr /= max + 1E-6;
        // Range clip just to be safe
        fl = Range.clip(fl, -1, 1);
        bl = Range.clip(bl, -1, 1);
        br = Range.clip(br, -1, 1);
        fr = Range.clip(fr, -1, 1);
        // Set powers
        leftDriveB.setPower(bl);
        leftDriveF.setPower(fl);
        rightDriveF.setPower(fr);
        rightDriveB.setPower(br);
    }

    /**
     * Sets the speeds of the motors to 0.
     */
    public void setSpeedZero () {
        setSpeedAll(0, 0, 0, 0, 0);
    }

    /**
     * Updates the current position (globalRobot) of the robot based off of the change in the
     * odometry encoders.
     */
    public void updatePosition () {
        bulkData = expansionHub.getBulkInputData();
        double currentL = (double) bulkData.getMotorCurrentPosition(lOWheel) / 1144.0;
        double currentC = (double) bulkData.getMotorCurrentPosition(cOWheel) / 1144.0;
        double currentR = (double) bulkData.getMotorCurrentPosition(rOWheel) / 1144.0;
        deltaL = currentL - previousL;
        deltaC = currentC - previousC;
        deltaR = currentR - previousR;
        previousL = currentL;
        previousC = currentC;
        previousR = currentR;

        globalRobot.updatePosition(deltaL, deltaC, deltaR);
    }
}