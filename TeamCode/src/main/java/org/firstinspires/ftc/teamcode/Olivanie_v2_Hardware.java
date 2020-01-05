package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class Olivanie_v2_Hardware
{


    /* Public OpMode members. */
    public DcMotorEx leftDriveF = null;
    public DcMotorEx rightDriveF = null;
    public DcMotorEx leftDriveB = null;
    public DcMotorEx rightDriveB = null;
    public DcMotorEx leftCollector = null;
    public DcMotor templeftCollector = null;
    public DcMotor rightCollector = null;
    public DcMotor tape = null;
    public Servo claw = null;
    public Servo foundation = null;
    public Servo left4Bar = null;
    public Servo right4Bar = null;
    public Servo gate = null;
    public Servo skystoneDumper = null;
    public Servo markerDumper = null;
    public Servo leftSideClaw = null;
    public Servo rightSideClaw = null;
    public CRServo tapeMeasure = null;
    public ColorSensor colorSensor = null;
    public DistanceSensor distanceSensor = null;

    public static final double HELD = 0.5;
    public static final double DROPPED = 0;
    public static final double GATE_OPEN = 0.05;
    public static final double GATE_CLOSED = 0.4;
    public static final double SKYSTONE_DUMPER_OPEN = 0.5;
    public static final double SKYSTONE_DUMPER_CLOSED = 0.9;
    public static final double RELEASEDL = .4;
    public static final double GRABBEDL = 0;
    public static final double RELEASEDR = 0.6;
    public static final double GRABBEDR = 1;
    public static final double BLOCK1 = .19;
    public static final double BLOCK2 = .27;
    public static final double BLOCK3 = .35;
    public static final double BLOCK4 = .44;
    public static final double DOWN = .9;
    public static final double [] ARMPOSITION = {0, BLOCK1, BLOCK2, BLOCK3, BLOCK4, DOWN};
    public static final double DISTANCE_BETWEEN_WHEELS = 15;

    double thetaL;
    double thetaC;
    double thetaR;

    Global_Robot globalRobot = new Global_Robot(0, 0, 0);

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
        leftDriveF = (DcMotorEx) hwMap.dcMotor.get("left drive f");
        rightDriveF = (DcMotorEx) hwMap.dcMotor.get("right drive f");
        leftDriveB = (DcMotorEx) hwMap.dcMotor.get("left drive b");
        rightDriveB = (DcMotorEx) hwMap.dcMotor.get("right drive b");
        templeftCollector = hwMap.dcMotor.get("left collector");
        rightCollector = hwMap.dcMotor.get("right collector");
        tape = hwMap.dcMotor.get("tape");

        leftCollector = (DcMotorEx)templeftCollector;

        // Define and initialize servos
        claw = hwMap.servo.get("claw");
        foundation = hwMap.servo.get("foundation");
        left4Bar = hwMap.servo.get("left 4 bar");
        right4Bar = hwMap.servo.get("right 4 bar");
        gate = hwMap.servo.get("gate");
        skystoneDumper = hwMap.servo.get("skystone dumper");
        markerDumper = hwMap.servo.get("son of brian");
        leftSideClaw = hwMap.servo.get("left side claw");
        rightSideClaw = hwMap.servo.get("right side claw");
        tapeMeasure = hwMap.crservo.get("tape measure");

        // Define and initialize sensors
        colorSensor = hwMap.colorSensor.get("color distance");
        distanceSensor = hwMap.get(DistanceSensor.class, "color distance");

        // Set direction for all motors
        leftDriveF.setDirection(DcMotor.Direction.FORWARD);
        rightDriveF.setDirection(DcMotor.Direction.REVERSE);
        leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);

        leftCollector.setDirection(DcMotor.Direction.FORWARD);
        rightCollector.setDirection(DcMotor.Direction.REVERSE);
        tape.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set all motors to zero power
        leftDriveF.setPower(0);
        rightDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveB.setPower(0);
        leftCollector.setPower(0);
        tape.setPower(0);

        // Set all servos to default positions
        openClaw();
        openFoundation();
        set4Bar(DOWN);
        gate.setPosition(GATE_CLOSED);
        skystoneDumper.setPosition(SKYSTONE_DUMPER_CLOSED);
        markerDumper.setPosition(HELD);
        leftSideClaw.setPosition(RELEASEDL);
        rightSideClaw.setPosition(RELEASEDR);
        tapeMeasure.setPower(0);

        // Set motors to use brake mode
        leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tape.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Odometry encoders on these
        leftCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tape.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // PIDF
        leftDriveF.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        rightDriveF.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        leftDriveB.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        rightDriveB.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        leftDriveF.setPositionPIDFCoefficients(10);
        rightDriveF.setPositionPIDFCoefficients(10);
        leftDriveB.setPositionPIDFCoefficients(10);
        rightDriveB.setPositionPIDFCoefficients(10);



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

    public void setGlobalRobot (double x, double y, double angle) {
        setX(x);
        setY(y);
        setAngle(angle);
    }

    public void setX (double x) {
        globalRobot.setX(x);
    }

    public void setY (double y) {
        globalRobot.setY(y);
    }

    public void setAngle (double angle) {
        globalRobot.setAngle(angle);
    }

    public double getXPos () {
        return globalRobot.getX();
    }

    public double getYPos () {
        return globalRobot.getY();
    }

    public double getWorldAngle_rad () {
        return globalRobot.getAngle();
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
        foundation.setPosition(0.35);
    }

    public void openFoundation() {
        foundation.setPosition(1);
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
        claw.setPosition(.35);
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
//        if (leftDriveF.getMode()      != DcMotor.RunMode.RUN_TO_POSITION ||
//                leftDriveB.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
//                rightDriveF.getMode()   != DcMotor.RunMode.RUN_TO_POSITION ||
//                rightDriveB.getMode()  != DcMotor.RunMode.RUN_TO_POSITION) {
            driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            // When the encoder is reset, also reset the target position, so it doesn't add an old
            // target position when using driveAddTargetPosition().
            driveSetTargetPosition(0, 0, 0, 0);
//        }
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
        leftDriveF.setTargetPosition(leftDriveF.getCurrentPosition()+flPosition);
        rightDriveF.setTargetPosition(rightDriveF.getCurrentPosition()+frPosition);
        leftDriveB.setTargetPosition(leftDriveB.getCurrentPosition()+blPosition);
        rightDriveB.setTargetPosition(rightDriveB.getCurrentPosition()+brPosition);
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

    boolean driveLeftAreBusy ()
    {
        return rightDriveF.isBusy() && leftDriveB.isBusy();
    }

    boolean driveRightAreBusy ()
    {
        return leftDriveF.isBusy() && rightDriveB.isBusy();
    }


    public void setPowerAll (double power) {
        leftDriveF.setPower(power);
        rightDriveF.setPower(power);
        leftDriveB.setPower(power);
        rightDriveB.setPower(power);
    }


    public void setPowerLeft (double power) {
        leftDriveF.setPower(power);
        leftDriveB.setPower(power);
    }

    public void setPowerRight (double power) {
        rightDriveF.setPower(power);
        rightDriveB.setPower(power);
    }

    public void setSpeedAll (double vX, double vY, double vA) {
        double fl = vY - vX - vA;
        double bl = vY + vX - vA;
        double br = vY - vX + vA;
        double fr = vY + vX + vA;
        double max = fl;
        max = Math.max(max, bl);
        max = Math.max(max, br);
        max = Math.max(max, fr);
        fl /= max * globalRobot.R;
        bl /= max * globalRobot.R;
        br /= max * globalRobot.R;
        fr /= max * globalRobot.R;
        fl = Range.clip(fl, -1, 1);
        bl = Range.clip(bl, -1, 1);
        br = Range.clip(br, -1, 1);
        fr = Range.clip(fr, -1, 1);
        leftDriveF.setPower(fl);
        leftDriveB.setPower(bl);
        rightDriveB.setPower(br);
        rightDriveF.setPower(fr);
    }

    public void updatePosition () {
        thetaL = ((double) rightCollector.getCurrentPosition() / 1304) - thetaL;
        thetaC = ((double) leftCollector.getCurrentPosition() / 1304) - thetaC;
        thetaR = ((double) tape.getCurrentPosition() / 1304) - thetaR;
        globalRobot.updatePosition(thetaL, thetaC, thetaR);
    }

    public Coordinate getCoordinate () {
        return globalRobot.getCoordinate();
    }

    public void grabStone () {

    }

    public void dropStone () {

    }
}