package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.Range
import java.lang.Math.abs

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

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common Olivanie hardware class to define the devices on the robot.
 * All device access is managed through the FBrian_Stormz_Hardware class.
 */

@TeleOp(name="Olivanie TeleOp 3.0", group="Olivanie")


class KOlivanieV3TeleOp: OpMode() {

        /* Declare OpMode members. */
//    use the class created to define Olivanie's hardware
    var robot: KRoboMovement= KRoboMovement()

    var x: Double= 0.0
    var y: Double= 0.0
    var z: Double= 0.0
    var inertia: Double= 0.5
    var time_a: Double= 0.0
    var dt: Double= 0.0
    var arm: Double= 0.9
    var hight: Int= 5
    var tapePower: Double= 0.0  // out is .32 in is 0.04
    var fingerL: Double= 1.0
    var fingerR: Double= 0.0
    var maxVelocityFL: Double= 0.0
    var maxVelocityFR: Double= 0.0
    var maxVelocityBL: Double= 0.0
    var maxVelocityBR: Double= 0.0
    var liftPosition: Double= 0.0
    var barPosition: Double= 0.0
    var collectOtronACTIVE: Boolean= false
    var collectOtronSWITCHING: Boolean= false
    var collectOtronREVERSE: Boolean= false
    var clawSWITCHING: Boolean= false
    var clawIsOpen: Boolean= false
    var capstone: Boolean= false
    var counter: Double= 101.0
    var stateLift: Int= 0
    var state2: Int= 0
    var barIn: Boolean= true
    var switcherLiftUp: Boolean= false
    var switcherLiftDown: Boolean= false
    var switcherBar: Boolean= false
    var switcherFoundation: Boolean= false

    // Code to run once when the driver hits INIT
    override fun init() {

        // Initialize the hardware variables.
        // The init() method of the hardware class does all the work here
        robot.init(hardwareMap)

        // The lift in in Run To Position
        robot.lift?.setTargetPosition(0)
        robot.lift?.setMode(DcMotor.RunMode.RUN_TO_POSITION)

        // Power up the lift. This stays powered the whole time.
        robot.lift?.setPower(1.0)

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Robot ready")

    }

    // Code to run in a loop after the driver hits play until they hit the stop button
    override fun loop() {

        // Calculate loop Period (dt).
        // Let's not repeat last year's failure/laziness that killed our performance at Regionals...
        dt = getRuntime() - time_a;
        time_a = getRuntime()
        telemetry.addData("Loop Time", "%f", dt);


        telemetry.addData("Arm", "%f", arm);
        telemetry.addData("Height", "%d", hight);
        telemetry.addData("Capstone", capstone);
        telemetry.addData("Tape Power", "%f", tapePower);

        robot.updatePosition();
        telemetry.addData("Odometry L", "%d", robot.leftCollector?.getCurrentPosition());
        telemetry.addData("Odometry R", "%d", robot.leftDriveB?.getCurrentPosition());
        telemetry.addData("Odometry C", "%d", robot.tape?.getCurrentPosition());
        telemetry.addData("X Position", "%f", robot.getX());
        telemetry.addData("Y Position", "%f", robot.getY());
        telemetry.addData("Angle", "%f", Math.toDegrees(robot.getWorldAngle_rad()));
        telemetry.addData("Theta L", "%f", robot.deltaL);
        telemetry.addData("Theta R", "%f", robot.deltaR);
        telemetry.addData("Theta C", "%f", robot.deltaC);
        telemetry.addData("Odometry R Raw", "%d", robot.tape?.getCurrentPosition());
        telemetry.addData("Lift State", "%d", stateLift);


        // Send telemetry message to signify robot running
        telemetry.addData("", "\"Well Met!\"");

        // Gamepad 1 is the driver, Gamepad 2 is the mechanisms

        // Drone Drive------------------------------------------------------------------------------

        y = if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1)
            (-gamepad1.left_stick_y).toDouble();
        else
            0.0

        x = if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1)
            gamepad1.left_stick_x.toDouble();
        else
            0.0

        z = if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1)
            gamepad1.right_stick_x*0.8
        else
            0.0

        var maxvalue: Double= abs(y + x - z)
        if (abs(y + x - z) > maxvalue) {
            maxvalue= abs(y + x - z)
        }
        if (abs(y - x + z) > maxvalue) {
            maxvalue= abs(y - x + z)
        }
        if (abs(y + x + z) > maxvalue) {
            maxvalue= abs(y + x + z)
        }
        if (abs(y - x - z) > maxvalue) {
            maxvalue= abs(y - x - z)
        }
        if (maxvalue < 1.0) {
            maxvalue = 1.0
        }

        var frontLeftPower: Double= (Range.clip(((y + x + z) / maxvalue), -1.0, 1.0))
        var frontRightPower: Double= (Range.clip(((y + x - z) / maxvalue), -1.0, 1.0))
        var backLeftPower: Double= (Range.clip(((y - x + z) / maxvalue), -1.0, 1.0))
        var backRightPower: Double= (Range.clip(((y - x - z) / maxvalue), -1.0, 1.0))

        if ((frontLeftPower > 0.1 || frontRightPower > 0.1 || backLeftPower > 0.1
                        || backRightPower > 0.1) || (frontLeftPower < -0.1 || frontRightPower < -0.1
                        || backLeftPower < -0.1 || backRightPower < -0.1))
        {
            inertia+= (0.7*dt)
            inertia= Range.clip(inertia, 0.0, 1.0)
        }else{
            inertia= 0.5
        }

        if (arm < 0.5) {
            // Slow down the robot when depositing
            inertia= 0.3
        }else if (inertia < 0.5){
            // Exit slow mode without waiting for inertia to build back up
            inertia= 0.5
        }


        robot.driveSetPower(frontLeftPower*inertia, backLeftPower*inertia, frontRightPower*inertia, backRightPower*inertia)
        // End Drive--------------------------------------------------------------------------------

        // Collector--------------------------------------------------------------------------------
        if (gamepad1.left_bumper || gamepad1.right_bumper)
            collectOtronSWITCHING = true
        else if (collectOtronSWITCHING) {
            collectOtronSWITCHING = false
            collectOtronACTIVE = !collectOtronACTIVE
        }

        // Check if we are reversed or not
        if (gamepad1.left_bumper)
            collectOtronREVERSE = true
        else if (gamepad1.right_bumper)
            collectOtronREVERSE = false

        // Run the Collector
        if (collectOtronACTIVE && !collectOtronREVERSE)
            robot.collectorOn()
        else if (collectOtronACTIVE)
            robot.collectorRev()
        else
            robot.collectorOff()
        // End Collector----------------------------------------------------------------------------

        // Claw-------------------------------------------------------------------------------------
        if (gamepad2.x && !clawSWITCHING) {
            clawSWITCHING = true
        }
        else if (!gamepad2.x && clawSWITCHING) {
            if (clawIsOpen) {
                clawIsOpen = false
                robot.closeClaw()
            }
            else {
                clawIsOpen = true
                robot.openClaw()
            }
            clawSWITCHING = false
        }
        // End Claw---------------------------------------------------------------------------------

        // Foundation-------------------------------------------------------------------------------
        if (gamepad1.a && !switcherFoundation) {
            switcherFoundation = true
        }
        else if (switcherFoundation && !gamepad1.a) {
            if (robot.foundation!!.getPosition() > 0.7f) {
                robot.openFoundation()
            }
            else {
                robot.closeFoundation()
            }
            switcherFoundation = false
        }
        // End Foundation---------------------------------------------------------------------------

        // Capstone---------------------------------------------------------------------------------
        if (gamepad2.b && !capstone) {
            capstone = true
        }
        else if (capstone && !gamepad2.b) {
            state2 ++
            capstone = false
        }
        if (state2 == 1) {
            if (robot.markerDumper!!.getPosition() > 0.2)
                robot.markerDumper?.setPosition(KOlivanieV3Hardware.DROPPED)
            else
                robot.markerDumper?.setPosition(KOlivanieV3Hardware.HELD)
            state2 = 0
        }
        // End Capstone-----------------------------------------------------------------------------

        // Tape Measure-----------------------------------------------------------------------------
        tapePower = Range.clip(gamepad2.right_trigger.toInt() - gamepad2.left_trigger.toInt(), -1,
                1).toDouble()
        robot.tape!!.setPower(-tapePower)
        // End Tape Measure-------------------------------------------------------------------------

        // 4-Bar------------------------------------------------------------------------------------
        if ((gamepad1.y || gamepad2.y) && !switcherBar) {
            switcherBar = true
        }
        if (!gamepad1.y && !gamepad2.y && switcherBar) {
            if (barIn)
                robot.place4Bar()
            else
                robot.close4Bar()
            barIn = !barIn
            switcherBar = false
        }
        // End 4-Bar--------------------------------------------------------------------------------

        // Lift-------------------------------------------------------------------------------------

        if ((gamepad1.dpad_up || gamepad2.dpad_up) && !switcherLiftUp && !switcherLiftDown) {
            switcherLiftUp = true
        }else if ((gamepad1.dpad_down || gamepad2.dpad_down) && !switcherLiftUp && !switcherLiftDown) {
            switcherLiftDown = true
        }else if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad2.dpad_up &&
                !gamepad2.dpad_down && switcherLiftUp) {
            stateLift++
            if (stateLift == KOlivanieV3Hardware.LIFTPOSITION.size) {
                stateLift--
                switcherLiftUp = false
            }
        }else if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad2.dpad_up &&
                !gamepad2.dpad_down && switcherLiftDown) {
            stateLift--
            if (stateLift < 0) {
                stateLift++
                switcherLiftDown = false
            }
        }

        robot.lift!!.setTargetPosition(KOlivanieV3Hardware.LIFTPOSITION[stateLift]);
        if (robot!!.lift?.getCurrentPosition()?.minus(robot!!.lift?.getTargetPosition()!!)?.let { Math.abs(it) }!! < 100)
            robot.lift!!.setPower(0.0)
        else
            robot!!.lift?.setPower(1.0)
        telemetry.addData("Lift motor power: ", robot!!.lift?.getPower())

        // End Lift---------------------------------------------------------------------------------

//        // Finger Test
//        if (gamepad1.left_stick_button)
//            fingerR += .01;
//        else if (gamepad1.right_stick_button)
//            fingerR -= .01;

        if (robot.leftDriveF!!.getVelocity() > maxVelocityFL) {
            maxVelocityFL = robot.leftDriveF!!.getVelocity()
        }
        if (robot.rightDriveF!!.getVelocity() > maxVelocityFR) {
            maxVelocityFR = robot.rightDriveF!!.getVelocity()
        }
        if (robot.leftDriveB!!.getVelocity() > maxVelocityBL) {
            maxVelocityBL = robot.leftDriveB!!.getVelocity()
        }
        if (robot.rightDriveB!!.getVelocity() > maxVelocityBR) {
            maxVelocityBR = robot.rightDriveB!!.getVelocity()
        }

        counter += dt
    }

}