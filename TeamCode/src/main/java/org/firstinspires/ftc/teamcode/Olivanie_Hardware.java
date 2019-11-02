package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class Olivanie_Hardware
{


    /* Public OpMode members. */
    public DcMotor leftDriveF = null;
    public DcMotor rightDriveF = null;
    public DcMotor leftDriveB = null;
    public DcMotor rightDriveB = null;
    public DcMotor leftCollector = null;
    public DcMotor rightCollector = null;
    public DcMotor arm = null;
    public Servo claw = null;
    public Servo leftFoundation = null;
    public Servo rightFoundation = null;


    /* Local OpMode members. */
    HardwareMap hwMap              = null;

    /* Constructor */
    public Olivanie_Hardware() {

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
        arm = hwMap.dcMotor.get("arm");

        // Define and initialize servos
        claw = hwMap.servo.get("claw");
        leftFoundation = hwMap.servo.get("left foundation");
        rightFoundation = hwMap.servo.get("right foundation");

        // Set direction for all motors
        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);

        leftCollector.setDirection(DcMotor.Direction.FORWARD);
        rightCollector.setDirection(DcMotor.Direction.REVERSE);

        arm.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        leftDriveF.setPower(0);
        rightDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveB.setPower(0);
        leftCollector.setPower(0);
        rightCollector.setPower(0);
        arm.setPower(0);

        // Set all servos to default positions
        claw.setPosition(1);
        openFoundation();

        // Set all motors to use brake mode
        leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // Set almost all motors to run with encoders
        leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    /**
     * FUNCTIONS
     */

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

    public double getLeftWheelEncoder () {
        return ((double) leftDriveF.getCurrentPosition() + (double) leftDriveB.getCurrentPosition())
                / 2;
    }

    public double getRightWheelEncoder () {
        return ((double) rightDriveF.getCurrentPosition()
                + (double) rightDriveB.getCurrentPosition()) / 2;
    }

    public void closeFoundation () {
        leftFoundation.setPosition(0);
        rightFoundation.setPosition(.5);
    }

    public void openFoundation () {
        leftFoundation.setPosition(.5);
        rightFoundation.setPosition(0);
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
     * driveSetMode sets all of the drive train motors to the specified positions.
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

    boolean driveAllAreBusy()
    {
        return leftDriveF.isBusy() && rightDriveF.isBusy() && leftDriveB.isBusy()
                && rightDriveB.isBusy();
    }


    void driveAddTargetPosition(int flPosition, int frPosition, int blPosition, int brPosition)
    {
        leftDriveF.setTargetPosition(leftDriveF.getTargetPosition()+flPosition);
        rightDriveF.setTargetPosition(rightDriveF.getTargetPosition()+frPosition);
        leftDriveB.setTargetPosition(leftDriveB.getTargetPosition()+blPosition);
        rightDriveB.setTargetPosition(rightDriveB.getTargetPosition()+brPosition);
    }
}