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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Olivanie TeleOp 3.0", group="Olivanie")
public class Olivanie_v3_TeleOp extends OpMode {

    /* Declare OpMode members. */

    // use the class created to define Olivanie's hardware
    RoboMovement robot = new RoboMovement();

    double  x = 0;
    double  y = 0;
    double  z = 0;

    double inertia      = 0.5;

    double time_a       = 0;
    double dt           = 0;

    int drop = 0;

    double fineAdjust = 0;

    double tapePower = 0;// out is .32 in is 0.04

    double fingerL = 1;
    double fingerR = 0;

    double maxVelocityFL = 0;
    double maxVelocityFR = 0;
    double maxVelocityBL = 0;
    double maxVelocityBR = 0;

    double liftPosition = 0;
    double barPosition = 0;

    double liftTimer = 0;

    boolean collectOtronACTIVE     = false;
    boolean collectOtronSWITCHING  = false;
    boolean collectOtronREVERSE    = false;
    boolean clawSWITCHING = false;
    boolean clawIsOpen = true;
    boolean capstone = false;
    boolean stackstate = true;

    double  counter = 101;
    int stateLift = 0;
    int state2 = 0;
    boolean barIn = true;
    boolean switcherLiftUp = false;
    boolean switcherLiftDown = false;
    boolean switcherDrop = false;
    boolean switcherFoundation = false;
    boolean switcherStack = false;

    // Code to run once when the driver hits INIT
    @Override
    public void init() {

         // Initialize the hardware variables.
         // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // The lift in in Run To Position
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Power up the lift. This stays powered the whole time.
        robot.lift.setPower(1);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Robot ready");

    }

    // Code to run in a loop after the driver hits play until they hit the stop button
    @Override
    public void loop() {

        // Calculate loop Period (dt).
        // Let's not repeat last year's failure/laziness that killed our performance at Regionals...
        // Units are seconds
        dt = getRuntime() - time_a;
        time_a = getRuntime();
        telemetry.addData("Loop Time", "%f", dt);

        telemetry.addData("Capstone", capstone);
        telemetry.addData("Tape Power", "%f", tapePower);

        robot.updatePosition();
        telemetry.addData("Odometry L", "%d", robot.leftCollector.getCurrentPosition());
        telemetry.addData("Odometry R", "%d", robot.leftDriveB.getCurrentPosition());
        telemetry.addData("Odometry C", "%d", robot.tape.getCurrentPosition());
        telemetry.addData("X Position", "%f", robot.getX());
        telemetry.addData("Y Position", "%f", robot.getY());
        telemetry.addData("Angle", "%f", Math.toDegrees(robot.getWorldAngle_rad()));
        telemetry.addData("Theta L", "%f", robot.deltaL);
        telemetry.addData("Theta R", "%f", robot.deltaR);
        telemetry.addData("Theta C", "%f", robot.deltaC);
        telemetry.addData("Odometry R Raw", "%d", robot.tape.getCurrentPosition());
        telemetry.addData("Lift State", "%d", stateLift);
        telemetry.addData("Drop", "%d", drop);


        // Send telemetry message to signify robot running
        telemetry.addData("", "\"Well Met!\"");

        // Gamepad 1 is the driver, Gamepad 2 is the mechanisms

        // Drone Drive------------------------------------------------------------------------------

            if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1)
                y = -gamepad1.left_stick_y;
            else
                y = 0;

            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1)
                x = gamepad1.left_stick_x;
            else
                x = 0;

            if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1)
                z = gamepad1.right_stick_x*0.8;
            else
                z = 0;

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

            double frontLeftPower  = (Range.clip(((y + x + z) / maxvalue), -1.0, 1.0));
            double frontRightPower = (Range.clip(((y + x - z) / maxvalue), -1.0, 1.0));
            double backLeftPower   = (Range.clip(((y - x + z) / maxvalue), -1.0, 1.0));
            double backRightPower  = (Range.clip(((y - x - z) / maxvalue), -1.0, 1.0));

            if ((frontLeftPower > 0.1 || frontRightPower > 0.1 || backLeftPower > 0.1
                    || backRightPower > 0.1) || (frontLeftPower < -0.1 || frontRightPower < -0.1
                    || backLeftPower < -0.1 || backRightPower < -0.1))
            {
                inertia += (0.7*dt);
                inertia = Range.clip(inertia, 0, 1);
            }
            else
            {
                inertia = 0.5;
            }

        if (inertia < 0.5) {
            // Exit slow mode without waiting for inertia to build back up
            inertia = 0.5;
        }


        robot.driveSetPower(frontLeftPower*inertia, backLeftPower*inertia,
                    frontRightPower*inertia, backRightPower*inertia);
        // End Drive--------------------------------------------------------------------------------

        // Collector--------------------------------------------------------------------------------
        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad2.left_bumper ||
                gamepad2.right_bumper)
            collectOtronSWITCHING = true;
        else if (collectOtronSWITCHING) {
            collectOtronSWITCHING = false;
            collectOtronACTIVE = !collectOtronACTIVE;
        }

        // Check if we are reversed or not
        if (gamepad1.left_bumper || gamepad2.left_bumper)
            collectOtronREVERSE = true;
        else if (gamepad1.right_bumper || gamepad2.right_bumper)
            collectOtronREVERSE = false;

        // Run the Collector
        if (collectOtronACTIVE && !collectOtronREVERSE)
            robot.collectorOn();
        else if (collectOtronACTIVE)
            robot.collectorRev();
        else
            robot.collectorOff();

        if (collectOtronACTIVE || !clawIsOpen)
            robot.kniod.setPosition(1);
        else
            robot.kniod.setPosition(0);
        // End Collector----------------------------------------------------------------------------

        // Claw-------------------------------------------------------------------------------------
        if (gamepad2.x && !clawSWITCHING) {
            clawSWITCHING = true;
        }
        else if (!gamepad2.x && clawSWITCHING) {
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
        // End Claw---------------------------------------------------------------------------------

        // Foundation-------------------------------------------------------------------------------
        if (gamepad1.a && !switcherFoundation) {
            switcherFoundation = true;
        }
        else if (switcherFoundation && !gamepad1.a) {
            if (robot.foundation.getPosition() > 0.7f) {
                robot.openFoundation();
            }
            else {
                robot.closeFoundation();
            }
            switcherFoundation = false;
        }
        // End Foundation---------------------------------------------------------------------------

        // Capstone---------------------------------------------------------------------------------

        // End Capstone-----------------------------------------------------------------------------

        // Tape Measure-----------------------------------------------------------------------------
        tapePower = Range.clip(gamepad1.right_trigger + gamepad2.right_trigger -
                        gamepad2.left_trigger - gamepad1.left_trigger, -1, 1);
        robot.tape.setPower(-tapePower);
        // End Tape Measure-------------------------------------------------------------------------

        // 4-Bar------------------------------------------------------------------------------------

        if (collectOtronACTIVE)
            robot.set4Bar(.85);
        else {
            if (Math.abs(robot.lift.getCurrentPosition() - robot.lift.getTargetPosition()) < 1000) {
                if (stateLift == 0 || !stackstate)
                    robot.close4Bar();
                else if (stateLift < 4)
                    robot.place4BarLow();
                else
                    robot.place4BarHigh();
            }
        }

        // End 4-Bar--------------------------------------------------------------------------------

        // Lift-------------------------------------------------------------------------------------

        if (gamepad2.dpad_up && !switcherLiftUp && !switcherLiftDown)
            switcherLiftUp = true;
        else if (gamepad2.dpad_down && !switcherLiftUp && !switcherLiftDown)
            switcherLiftDown = true;
        else if (!gamepad2.dpad_up && !gamepad2.dpad_down && switcherLiftUp) {
            stateLift++;
            stackstate = true;
            if (stateLift == robot.LIFTPOSITION.length)
                stateLift--;
            switcherLiftUp = false;
        }
        else if (!gamepad2.dpad_up && !gamepad2.dpad_down && switcherLiftDown) {
            stateLift--;
            if (stateLift < 0)
                stateLift++;
            switcherLiftDown = false;
        }

        if (gamepad2.b && !switcherStack)
            switcherStack = true;
        else if (!gamepad2.b && switcherStack) {
            fineAdjust = 0;
            stackstate = false;
            switcherStack = false;
        }

        if (gamepad2.a && !switcherDrop)
            switcherDrop = true;
        else if (!gamepad2.a && switcherDrop) {
            if (drop == 0)
                drop = 300;
            else
                drop = 0;
            switcherDrop = false;
        }

        if (stateLift == 0)
            drop = 0;

        fineAdjust += gamepad2.left_stick_y * 100 * dt;

        if (stackstate)
            robot.lift.setTargetPosition(robot.LIFTPOSITION[stateLift] - drop + (int)fineAdjust);
        else {
            liftTimer += dt;
            if (liftTimer > 1) {
                robot.lift.setTargetPosition(robot.LIFTPOSITION[0]);
                liftTimer = 0;
            }

        }

        if (Math.abs(robot.lift.getCurrentPosition() - robot.lift.getTargetPosition()) < 10)
            robot.lift.setPower(0);
        else
            robot.lift.setPower(1);
        telemetry.addData("Lift motor power: ", robot.lift.getPower());

        // End Lift---------------------------------------------------------------------------------
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
