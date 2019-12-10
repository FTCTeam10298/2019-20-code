package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class Olivanie_v2_Hardware
{


    /* Public OpMode members. */
    public DcMotor leftDriveF = null;
    public DcMotor rightDriveF = null;
    public DcMotor leftDriveB = null;
    public DcMotor rightDriveB = null;
    public DcMotor leftCollector = null;
    public DcMotor rightCollector = null;
    public Servo claw = null;
    public Servo leftFoundation = null;
    public Servo rightFoundation = null;
    public Servo left4Bar = null;
    public Servo right4Bar = null;
    public Servo gate = null;
    public Servo skystoneDumper = null;
    public Servo markerDumper = null;
    public ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;

    public static final double CLOSED = 0.1;
    public static final double OPEN = 1;
    public static final double HELD = 0;
    public static final double DOWN = .9;
    public static final double GATE_OPEN = 0.05;
    public static final double GATE_CLOSED = 0.4;
    public static final double SKYSTONE_DUMPER_OPEN = 0.6;
    public static final double SKYSTONE_DUMPER_CLOSED = 1;
    public static final double BLOCK1 = .1;
    public static final double BLOCK2 = .2;
    public static final double BLOCK3 = .3;
    public static final double BLOCK4 = .4;
    public static final double [] ARMPOSITION = {DOWN, BLOCK1, BLOCK2, BLOCK3, BLOCK4};

    RoboPoint roboPoint = new RoboPoint();

    /* Local OpMode members. */
    HardwareMap hwMap              = null;

    /* Constructor */
    public Olivanie_v2_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        leftDriveF = hwMap.dcMotor.get("left drive f");
        rightDriveF = hwMap.dcMotor.get("right drive f");
        leftDriveB = hwMap.dcMotor.get("left drive b");
        rightDriveB = hwMap.dcMotor.get("right drive b");
        leftCollector = hwMap.dcMotor.get("left collector");
        rightCollector = hwMap.dcMotor.get("right collector");

        // Define and initialize servos
        claw = hwMap.servo.get("claw");
        leftFoundation = hwMap.servo.get("left foundation");
        rightFoundation = hwMap.servo.get("right foundation");
        left4Bar = hwMap.servo.get("left 4 bar");
        right4Bar = hwMap.servo.get("right 4 bar");
        gate = hwMap.servo.get("gate");
        skystoneDumper = hwMap.servo.get("skystone dumper");
        markerDumper = hwMap.servo.get("son of brian");

        // Define and initialize sensors
        colorSensor = hwMap.colorSensor.get("color distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "color distance");

        // Set direction for all motors
        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);

        leftCollector.setDirection(DcMotor.Direction.FORWARD);
        rightCollector.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        leftDriveF.setPower(0);
        rightDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveB.setPower(0);
        leftCollector.setPower(0);

        // Set all servos to default positions
        openClaw();
        openFoundation();
        set4Bar(DOWN);
        gate.setPosition(GATE_CLOSED);
        skystoneDumper.setPosition(SKYSTONE_DUMPER_CLOSED);
        markerDumper.setPosition(HELD);

        // Set motors to use brake mode
        leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use coast on these
        leftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run with encoders
        leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // No encoders on these
        leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    /**
     * FUNCTIONS
     */

    /**
     * driveSetPowerAll sets all of the drive train motors to the specified power level.
     * @param power Power level to set all motors to
     */
    public void driveSetPowerAll (double power)
    {
        driveSetPower(power, power, power, power);
    }

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

    public void setRoboPoint (double x, double y, double angle) {
        setX(x);
        setY(y);
        setAngle(angle);
    }

    public void setX (double x) {
        roboPoint.setX(x);
    }

    public void setY (double y) {
        roboPoint.setY(y);
    }

    public void setAngle (double angle) {
        roboPoint.setAngle(angle);
    }

    public double getXPos () {
        return roboPoint.getX();
    }

    public double getYPos () {
        return roboPoint.getY();
    }

    public double getWorldAngle_rad () {
        return Math.toRadians(roboPoint.getAngle());
    }

    public double getLeftWheelEncoder () {
        return ((double) leftDriveF.getCurrentPosition() + (double) leftDriveB.getCurrentPosition())
                / 2.0;
    }

    public double getRightWheelEncoder () {
        return ((double) rightDriveF.getCurrentPosition()
                + (double) rightDriveB.getCurrentPosition()) / 2.0;
    }

    public void closeFoundation() {
        leftFoundation.setPosition(.6);
        rightFoundation.setPosition(1);
    }

    public void openFoundation() {
        leftFoundation.setPosition(1);
        rightFoundation.setPosition(0);
    }

    public void runCollector (double power) {
        leftCollector.setPower(power);
        rightCollector.setPower(power);
    }

    public void collectorOn () {
        runCollector(-1);
    }

    public void collectorOff () {
        runCollector(0);
    }

    public void  collectorRev () {
        runCollector(1);
    }

    public void set4Bar (double position) {
        left4Bar.setPosition(position);
        right4Bar.setPosition(1 - position);
    }

    public void openClaw () {
        claw.setPosition(.5);
    }

    public void closeClaw() {
        claw.setPosition(1);
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

    void driveSetRunToPosition()
    {
        if (leftDriveF.getMode()      != DcMotor.RunMode.RUN_TO_POSITION ||
                leftDriveB.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
                rightDriveF.getMode()   != DcMotor.RunMode.RUN_TO_POSITION ||
                rightDriveB.getMode()  != DcMotor.RunMode.RUN_TO_POSITION) {
            driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            // When the encoder is reset, also reset the target position, so it doesn't add an old
            // target position when using driveAddTargetPosition().
            driveSetTargetPosition(0, 0, 0, 0);
        }
    }

    /**
     * driveSetTargetPosition sets all of the drive train motors to the specified positions.
     * @param flPosition Position to set front left motor to run to
     * @param frPosition Position to set front right motor to run to
     * @param blPosition Position to set back left motor to run to
     * @param brPosition Position to set back right motor to run to
     */
    void driveSetTargetPosition(int flPosition, int frPosition, int blPosition, int brPosition)
    {
        leftDriveF.setTargetPosition(flPosition);
        rightDriveF.setTargetPosition(frPosition);
        leftDriveB.setTargetPosition(blPosition);
        rightDriveB.setTargetPosition(brPosition);
    }

    void driveAddTargetPosition(int flPosition, int frPosition, int blPosition, int brPosition)
    {
        leftDriveF.setTargetPosition(leftDriveF.getTargetPosition()+flPosition);
        rightDriveF.setTargetPosition(rightDriveF.getTargetPosition()+frPosition);
        leftDriveB.setTargetPosition(leftDriveB.getTargetPosition()+blPosition);
        rightDriveB.setTargetPosition(rightDriveB.getTargetPosition()+brPosition);
    }

    boolean driveAnyReachedTarget() {
        return ((Math.abs(leftDriveF.getCurrentPosition() - leftDriveF.getTargetPosition()) > 50.0)
                && (Math.abs(leftDriveB.getCurrentPosition() - leftDriveB.getTargetPosition()) > 50.0)
                && (Math.abs(rightDriveF.getCurrentPosition() - rightDriveB.getTargetPosition()) > 50.0)
                && (Math.abs(rightDriveB.getCurrentPosition() - rightDriveB.getTargetPosition()) > 50.0));
    }

    boolean driveAllAreBusy()
    {
        return leftDriveF.isBusy() && rightDriveF.isBusy() && leftDriveB.isBusy()
                && rightDriveB.isBusy();
    }
}