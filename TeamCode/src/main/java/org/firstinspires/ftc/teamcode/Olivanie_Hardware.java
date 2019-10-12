package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
//    public DcMotor leftCollector = null;
//    public DcMotor rightCollector = null;


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
//        leftCollector = hwMap.dcMotor.get("left collector");
//        rightCollector = hwMap.dcMotor.get("right collector");

        // Set direction for all motors
        leftDriveF.setDirection(DcMotor.Direction.REVERSE);
        rightDriveF.setDirection(DcMotor.Direction.FORWARD);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);

//        leftCollector.setDirection(DcMotor.Direction.FORWARD);
//        rightCollector.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        leftDriveF.setPower(0);
        rightDriveF.setPower(0);
        leftDriveB.setPower(0);
        rightDriveB.setPower(0);
//        leftCollector.setPower(0);
//        rightCollector.setPower(0);


        // Set all motors to use brake mode
        leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // Set almost all motors to run with encoders
        leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
}