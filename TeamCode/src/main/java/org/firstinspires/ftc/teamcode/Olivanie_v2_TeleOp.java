/*
Copyright (c) 2016-20, FTC team #10298 Brain Stormz

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@Deprecated
@Disabled
public class Olivanie_v2_TeleOp extends OpMode {

    /* Declare OpMode members. */

    // use the class created to define Olivanie's hardware
    RoboMovement robot = new RoboMovement();

    double  x = 0;
    double  y = 0;
    double  z = 0;

    double inertia      = 0.5;

    double time_a       = 0;
    double dt           = 0;

    double arm = 0.9;

    int height = 5;

    double tapePower = 0;// out is .32 in is 0.04

    double fingerL = 1;
    double fingerR = 0;

    double maxVelocityFL = 0;
    double maxVelocityFR = 0;
    double maxVelocityBL = 0;
    double maxVelocityBR = 0;

    boolean collectOtronACTIVE     = false;
    boolean collectOtronSWITCHING  = false;
    boolean collectOtronREVERSE    = false;
    boolean clawSWITCHING = false;
    boolean clawIsOpen = true;
    boolean armChangeDown = false;
    boolean armChangeUp = false;
    boolean armMoving = false;
    boolean capstone = false;

    double  counter = 101;
    int state = 0;
    int state2 = 0;
    boolean switcher3 = false;
    boolean switcher4 = false;

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
        telemetry.addData("Arm", "%f", arm);
        telemetry.addData("Height", "%d", height);
        telemetry.addData("Capstone", capstone);
        telemetry.addData("Tape Power", "%f", tapePower);

        robot.updatePosition();
        telemetry.addData("Odometry L", "%d", robot.rightCollector.getCurrentPosition());
        telemetry.addData("Odometry R", "%d", robot.tape.getCurrentPosition());
        telemetry.addData("Odometry C", "%d", robot.leftCollector.getCurrentPosition());
        telemetry.addData("X Position", "%f", robot.getX());
        telemetry.addData("Y Position", "%f", robot.getY());
        telemetry.addData("Angle", "%f", Math.toDegrees(robot.getWorldAngle_rad()));
        telemetry.addData("Theta L", "%f", robot.deltaL);
        telemetry.addData("Theta R", "%f", robot.deltaR);
        telemetry.addData("Theta C", "%f", robot.deltaC);
        telemetry.addData("Odometry R Raw", "%d", robot.tape.getCurrentPosition());
        telemetry.addData("Finger L", "%f", fingerL);
        telemetry.addData("Finger R", "%f", fingerR);


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

            double frontLeftPower  = (Range.clip(((y - x + z) / maxvalue), -1.0, 1.0));
            double frontRightPower = (Range.clip(((y + x - z) / maxvalue), -1.0, 1.0));
            double backLeftPower   = (Range.clip(((y + x + z) / maxvalue), -1.0, 1.0));
            double backRightPower  = (Range.clip(((y - x - z) / maxvalue), -1.0, 1.0));

            if ((frontLeftPower > 0.1 || frontRightPower > 0.1 || backLeftPower > 0.1 || backRightPower > 0.1)
                    || (frontLeftPower < -0.1 || frontRightPower < -0.1 || backLeftPower < -0.1 || backRightPower < -0.1))
            {
                inertia += (0.7*dt);
                inertia = Range.clip(inertia, 0, 1);
            }
            else
            {
                inertia = 0.5;
            }

        if (arm < 0.5) {
            // Slow down the robot when depositing
            inertia = 0.3;
        }
        else if (inertia < 0.5) {
            // Exit slow mode without waiting for inertia to build back up
            inertia = 0.5;
        }


        robot.driveSetPower(frontLeftPower*inertia, backLeftPower*inertia,
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
        else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            collectOtronREVERSE = false;
        }

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
//
//        // Dumper
//        if ((gamepad1.y || gamepad2.y) && !switcher3) {
//            switcher3 = true;
//        }
//        else if (switcher3 && !gamepad1.y && !gamepad2.y) {
//            if (robot.gate.getPosition() > 0.3f) {
//                counter = 0;
//                arm = .6;
//                state++;
//            }
//            else {
//                counter = 0;
//                robot.skystoneDumper.setPosition(robot.SKYSTONE_DUMPER_CLOSED);
//                robot.gate.setPosition(robot.GATE_CLOSED);
//                state--;
//            }
//            switcher3 = false;
//        }
//
//        if (counter >= .5 && state == 1) {
//            robot.gate.setPosition(robot.GATE_OPEN);
//            state++;
//        }
//        else if (counter >= .5 && state == -1) {
//            state--;
//        }
//
//        if (counter >= .55 && state == 2) {
//            robot.skystoneDumper.setPosition(robot.SKYSTONE_DUMPER_OPEN);
//            state = 0;
//        }
//        else if (counter >= .55 && state == -2) {
//            arm = 0.9;
//            state = 0;
//        }
//        // End Dumper

        // Foundation
        if ((gamepad1.a || gamepad2.a) && !switcher4) {
            switcher4 = true;
        }
        else if (switcher4 && !gamepad1.a && !gamepad2.a) {
            if (robot.foundation.getPosition() < 0.7f) {
                robot.openFoundation();
            }
            else {
                robot.closeFoundation();
            }
            switcher4 = false;
        }
        // End Foundation

        // Capstone
        if ((gamepad1.b || gamepad2.b) && !capstone) {
            capstone = true;
        }
        else if (capstone && !gamepad1.b && !gamepad2.b) {
            state2 ++;
            capstone = false;
        }
        if (state2 == 1) {
//            if (robot.markerDumper.getPosition() > 0.2)
//                robot.markerDumper.setPosition(robot.DROPPED);
//            else
//                robot.markerDumper.setPosition(robot.HELD);
//            state2 = 0;
        }
        // End Capstone

        // Tape Measure
        tapePower = Range.clip(gamepad1.right_trigger + gamepad2.right_trigger
                - gamepad1.left_trigger - gamepad2.left_trigger, -1, 1);
        robot.tape.setPower(-tapePower);

        // Arm
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            arm += .2 * dt;
        }
        else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            arm -= .2 * dt;
        }

        if ((gamepad1.dpad_down || gamepad2.dpad_down) && !armChangeDown) {
            armChangeDown = true;
        }
        else if (!(gamepad1.dpad_down || gamepad2.dpad_down) && armChangeDown) {
            height--;
            armChangeDown = false;
            armMoving = true;
        }

        if ((gamepad1.dpad_up || gamepad2.dpad_up) && !armChangeUp) {
            armChangeUp = true;
        }
        else if (!(gamepad1.dpad_up || gamepad2.dpad_up) && armChangeUp) {
            height++;
            armChangeUp = false;
            armMoving = true;
            if (height == 5)
                robot.openClaw();
        }

        if (height < 1) {
            height = 1; //4
        }
        else if (height > 5) {
            height = 5; //0
        }

        if (armMoving) {
            //arm = robot.ARMPOSITION[height];
            armMoving = false;
        }

        robot.set4Bar(arm);
        // End Arm
//
//        // Finger Test
//        if (gamepad1.left_stick_button)
//            fingerR += .01;
//        else if (gamepad1.right_stick_button)
//            fingerR -= .01;

        if (robot.leftDriveF.getVelocity() > maxVelocityFL)
            maxVelocityFL = robot.leftDriveF.getVelocity();
        if (robot.rightDriveF.getVelocity() > maxVelocityFR)
            maxVelocityFR = robot.rightDriveF.getVelocity();
        if (robot.leftDriveB.getVelocity() > maxVelocityBL)
            maxVelocityBL = robot.leftDriveB.getVelocity();
        if (robot.rightDriveB.getVelocity() > maxVelocityBR)
            maxVelocityBR = robot.rightDriveB.getVelocity();

        counter += dt;
    }
}
