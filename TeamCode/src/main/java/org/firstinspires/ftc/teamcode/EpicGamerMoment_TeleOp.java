/*
Copyright (c) 2016-19, FTC team #10298 Brain Stormz

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Brain Stormz nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common Brian Stormz hardware class to define the devices on the robot.
 * All device access is managed through the FBrian_Stormz_Hardware class.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="EpicGamerMoment TeleOp", group="Gamer")
public class EpicGamerMoment_TeleOp extends OpMode {

    /* Declare OpMode members. */
    EpicGamerMoment_Hardware robot = new EpicGamerMoment_Hardware(); // use the class created to define Olivanie's hardware
    Robosition position = new Robosition(0, 0, 0);


    double leftPower = 0;
    double rightPower = 0;
    double drive = 0;
    double turn = 0;
    double max = 0;
    boolean collectorOn = false;
    boolean switcher1 = false;
    boolean switcher2 = false;
    boolean tankDrive = true;

    // Code to run once when the driver hits INIT
    @Override
    public void init() {

         // Initialize the hardware variables.
         // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "\"Watch your back\" \"I always do\" ");

    }

    // Code to run in a loop after the driver hits play until they hit the stop button
    @Override
    public void loop() {


        if (tankDrive) {
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
        }
        else {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.right_stick_y;
            turn  =  gamepad1.left_stick_x;

            // Combine drive and turn for blended motion.
            leftPower  = drive + turn;
            rightPower = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0)
            {
                leftPower /= max;
                rightPower /= max;
            }

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
        }

        if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0
                && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0
                && gamepad1.start && !switcher2) {
            switcher2 = true;
        }
        if (switcher2 && !gamepad1.start) {
            tankDrive = !tankDrive;
            switcher2 = false;
        }

        if (gamepad1.right_bumper && !switcher1)
            switcher1 = true;
        if (switcher1 && !gamepad1.right_bumper) {
            collectorOn = true;
            switcher1 = false;
        }
        if (gamepad1.right_bumper && collectorOn)
            switcher1 = true;
        if (collectorOn && switcher1) {
            switcher1 = false;
            collectorOn = false;
        }

//        if (collectorOn){
//            robot.leftCollector.setPower(1);
//            robot.rightCollector.setPower(1);
//        }

        telemetry.addData("leftPower: ",  "%.2f", leftPower);
        telemetry.addData("rightPower: ",  "%.2f", rightPower);
        telemetry.addData("Tank Drive? ", tankDrive);
        telemetry.addData("Current Angle: ", "%.2f", position.getAngle());
        telemetry.addData("X: ", "%.2f", position.getX());
        telemetry.addData("Y: ", "%.2f", position.getY());
        telemetry.update();


    }

    /**
    FUNCTIONS------------------------------------------------------------------------------------------------------
     */

}
