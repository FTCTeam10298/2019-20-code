package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class EpicGamerMoment_Hardware
{


    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
//    public DcMotor leftCollector = null;
//    public DcMotor rightCollector = null;


    /* Local OpMode members. */
    HardwareMap hwMap              = null;

    /* Constructor */
    public EpicGamerMoment_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        leftDrive = hwMap.dcMotor.get("left_drive");
        rightDrive = hwMap.dcMotor.get("right_drive");
//        leftCollector = hwMap.dcMotor.get("left_collector");
//        rightCollector = hwMap.dcMotor.get("right_collector");

        // Set direction for all motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftCollector.setDirection(DcMotor.Direction.FORWARD);
//        rightCollector.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
//        leftCollector.setPower(0);
//        rightCollector.setPower(0);


        // Set all motors to use brake mode
//        leftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Set almost all motors to run with encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    /**
     * FUNCTIONS
     */
}

