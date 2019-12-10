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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common Olivanie hardware class to define the devices on the robot.
 * All device access is managed through the FBrian_Stormz_Hardware class.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Olivanie TeleOp 2.0", group="Olivanie")
public class Olivanie_v2_TeleOp extends OpMode {

    /* Declare OpMode members. */

    // use the class created to define Olivanie's hardware
    Olivanie_v2_Hardware robot = new Olivanie_v2_Hardware();

    double  x = 0;
    double  y = 0;
    double  z = 0;

    double inertia      = 0.5;

    double time_a       = 0;
    double dt           = 0;

    double arm = 0;

    int height = 0;

    boolean collectOtronACTIVE     = false;
    boolean collectOtronSWITCHING  = false;
    boolean collectOtronREVERSE    = false;
    boolean clawSWITCHING = false;
    boolean clawIsOpen = true;
    boolean armChangeDown = false;
    boolean armChangeUp = false;
    boolean armMoving = false;


    // Code to run once when the driver hits INIT
    @Override
    public void init() {

         // Initialize the hardware variables.
         // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Robot ready");

    }

    // Code to run in a loop after the driver hits play until they hit the stop button
    @Override
    public void loop() {

        // Calculate loop Period (dt).
        // Let's not repeat last year's failure/laziness that killed our performance at Regionals...
        dt = getRuntime() - time_a;
        time_a = getRuntime();
        telemetry.addData("Loop Time", "%f", dt);
        telemetry.addData("Inertia", "%f", inertia);
        telemetry.addData("Arm", "%f", arm);
        telemetry.addData("Height", "%d", height);

        // Send telemetry message to signify robot running
        telemetry.addData("Say", "N8 is the gr8est without deb8");

        // Drone drive

            if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) {
                y = -gamepad1.left_stick_y;
            } else if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {
                y = -gamepad2.left_stick_y;
            }
            else {
                y = 0;
            }

            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1) {
                x = -gamepad1.left_stick_x;
            } else if (gamepad2.left_stick_x > .1 || gamepad2.left_stick_x < -.1) {
                x = -gamepad2.left_stick_x;
            }
            else {
                x = 0;
            }

            if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
                z = gamepad1.right_stick_x*0.8;
            } else if (gamepad2.right_stick_x > .1 || gamepad2.right_stick_x < -.1) {
                z = gamepad2.right_stick_x*0.8;
            }
            else {
                z = 0;
            }

            double maxvalue = abs(y + x - z);
            if (abs(y + x - z) > maxvalue) {
                maxvalue = abs(y + x - z);
            }
            if (abs(y - x + z) > maxvalue) {
                maxvalue = abs(y - x + z);
            }
            if (abs(y + x + z) > maxvalue) {
                maxvalue = abs(y + x + z);
            }
            if (abs(y - x - z) > maxvalue) {
                maxvalue = abs(y - x - z);
            }
            if (maxvalue < 1.0) {
                maxvalue = 1;
            }

            double frontLeftPower  = (-1 * Range.clip(((y - x + z) / maxvalue), -1.0, 1.0));
            double frontRightPower = (-1 * Range.clip(((y + x - z) / maxvalue), -1.0, 1.0));
            double backLeftPower   = (-1 * Range.clip(((y + x + z) / maxvalue), -1.0, 1.0));
            double backRightPower  = (-1 * Range.clip(((y - x - z) / maxvalue), -1.0, 1.0));

            if ((frontLeftPower > 0.1 || frontRightPower > 0.1 || backLeftPower > 0.1 || backRightPower > 0.1)
                    || (frontLeftPower < -0.1 || frontRightPower < -0.1 || backLeftPower < -0.1 || backRightPower < -0.1))
            {
                inertia += (0.6*dt);
                inertia = Range.clip(inertia, 0, 1);
            }
            else
            {
                inertia = 0.4;
            }

            robot.setPowerEach(frontLeftPower*inertia, backLeftPower*inertia,
                    frontRightPower*inertia, backRightPower*inertia);

        // Collector

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad2.left_bumper
                || gamepad2.right_bumper)
            collectOtronSWITCHING = true;
        else if (collectOtronSWITCHING) {
            collectOtronSWITCHING = false;
            if (collectOtronACTIVE)
                collectOtronACTIVE = false;
            else
                collectOtronACTIVE = true;
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper)
            collectOtronREVERSE = true;
        else if (gamepad1.right_bumper || gamepad2.right_bumper)
            collectOtronREVERSE = false;

        if (collectOtronACTIVE && !collectOtronREVERSE) {
            robot.collectorOn();
        }

        else if (collectOtronACTIVE)
            robot.collectorRev();
        else
            robot.collectorOff();

        // End Collector
        // Claw

        if ((gamepad1.x || gamepad2.x) && !clawSWITCHING) {
            clawSWITCHING = true;
        }
        else if (!(gamepad1.x || gamepad2.x) && clawSWITCHING) {
            if (clawIsOpen) {
                clawIsOpen = false;
                robot.closeClaw();
            }
            else {
                clawIsOpen = true;
                robot.openClaw();
            }
            clawSWITCHING = false;
        }

        // End Claw
        // Arm

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            arm += .1 * dt;
        }
        else if (gamepad1.dpad_down || gamepad2.dpad_down) {
            arm -= .1 * dt;
        }

        if ((gamepad1.a || gamepad2.a) && !armChangeDown) {
            armChangeDown = true;
        }
        else if (!(gamepad1.a || gamepad2.a) && armChangeDown) {
            height--;
            armChangeDown = false;
            armMoving = true;
        }

        if ((gamepad1.y || gamepad2.y) && !armChangeUp) {
            armChangeUp = true;
        }
        else if (!(gamepad1.y || gamepad2.y) && armChangeUp) {
            height++;
            armChangeUp = false;
            armMoving = true;
        }

        if (height < 0) {
            height = 4;
        }
        else if (height > 4) {
            height = 0;
        }

        if (armMoving) {
            arm = robot.ARMPOSITION[height];
            armMoving = false;
        }

        robot.set4Bar(arm);

        // End Arm

    }

}

    /**
    FUNCTIONS---------------------------------------------------------------------------------------
     */