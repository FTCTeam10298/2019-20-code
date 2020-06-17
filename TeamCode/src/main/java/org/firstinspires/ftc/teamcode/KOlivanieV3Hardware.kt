package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Servo.Direction.*
import com.qualcomm.robotcore.util.Range

import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

open class KOlivanieV3Hardware {
    companion object {
        val HELD = 0.32
        val DROPPED = 0.04
        val RELEASEDL = 0.0
        val GRABBEDL = 1.0
        val RELEASEDR = 1.0
        val GRABBEDR = 0.0
        val FINGERGRABBEDR = 1.0
        val FINGERRELEASER = 0.2
        val FINGERGRABBEDL = 0.0
        val FINGERRELEASEL = 0.8
        val BLOCK1 = -1000
        val BLOCK2 = -2000
        val BLOCK3 = -3000
        val BLOCK4 = -4000
        val BLOCK5 = -5000
        val BLOCK6 = -6000
        val BLOCK7 = -7000
        val BLOCK8 = -8000
        val LIFTPOSITION = arrayOf(0, BLOCK1, BLOCK2, BLOCK3, BLOCK4, BLOCK5, BLOCK6, BLOCK7, BLOCK8)
    }


    /* Public OpMode members. */
    var leftDriveF:DcMotorEx? = null
    var rightDriveF:DcMotorEx? = null
    var leftDriveB:DcMotorEx? = null
    var rightDriveB:DcMotorEx? = null
    var leftCollector:DcMotor? = null
    var rightCollector:DcMotor? = null
    var tape:DcMotor? = null
    var lift:DcMotor? = null
    var claw:Servo? = null
    var foundation:Servo? = null
    var left4Bar:Servo? = null
    var right4Bar:Servo? = null
    var markerDumper:Servo? = null
    var leftSideClaw:Servo? = null
    var rightSideClaw:Servo? = null
    var leftFinger:Servo? = null
    var rightFinger:Servo? = null

    var deltaL: Double = 0.0
    var deltaC: Double = 0.0
    var deltaR: Double = 0.0
    var previousL: Double = 0.0
    var previousC: Double = 0.0
    var previousR: Double = 0.0


    var globalRobot:KGlobalRobot = KGlobalRobot(0.0, 0.0, 180.0)

    //    Local OpMode members
    var hwMap:HardwareMap?= null

    //    Bulk data
    var motor3:ExpansionHubMotor?= null
    var lOWheel:ExpansionHubMotor= hwMap!!.dcMotor.get("left collector") as ExpansionHubMotor
    var rOWheel:ExpansionHubMotor= hwMap!!.dcMotor.get("left drive b")as ExpansionHubMotor
    var cOWheel:ExpansionHubMotor= hwMap!!.dcMotor.get("tape")as ExpansionHubMotor
    var expansionHub:ExpansionHubEx= hwMap!!.get(ExpansionHubEx.class ,"Expansion Hub 5")
    var bulkData:RevBulkData= expansionHub.getBulkInputData()

    /* Constructor */
    fun KOlivanieV3Hardware() {
    }

    /* Initialize standard Hardware interfaces */
    fun init(ahwMap:HardwareMap) {
        //    Save reference to Hardware map
        hwMap = ahwMap
        // Define and initialize motors
        leftDriveF = hwMap!!.dcMotor.get("left drive f") as DcMotorEx
        rightDriveF = hwMap!!.dcMotor.get("right drive f") as DcMotorEx
        leftDriveB = hwMap!!.dcMotor.get("left drive b") as DcMotorEx
        rightDriveB = hwMap!!.dcMotor.get("right drive b") as DcMotorEx
        leftCollector = hwMap!!.dcMotor.get("left collector")
        rightCollector = hwMap!!.dcMotor.get("right collector")
        tape = hwMap!!.dcMotor.get("tape")
        lift = hwMap!!.dcMotor.get("lift")

        // Define and initialize servos
        claw = hwMap!!.servo.get("claw")
        foundation = hwMap!!.servo.get("foundation")
        left4Bar = hwMap!!.servo.get("left 4 bar")
        right4Bar = hwMap!!.servo.get("right 4 bar")
        markerDumper = hwMap!!.servo.get("son of brian")
        leftSideClaw = hwMap!!.servo.get("left side claw")
        rightSideClaw = hwMap!!.servo.get("right side claw")
        leftFinger = hwMap!!.servo.get("left finger")
        rightFinger = hwMap!!.servo.get("right finger")

        // Define and initialize sensors

        // Set direction for all motors
        leftDriveF!!.setDirection(DcMotorSimple.Direction.REVERSE)
        rightDriveF!!.setDirection(DcMotorSimple.Direction.FORWARD)
        leftDriveB!!.setDirection(DcMotorSimple.Direction.REVERSE)
        rightDriveB!!.setDirection(DcMotorSimple.Direction.FORWARD)

        leftCollector!!.setDirection(DcMotorSimple.Direction.FORWARD)
        rightCollector!!.setDirection(DcMotorSimple.Direction.REVERSE)
        tape!!.setDirection(DcMotorSimple.Direction.FORWARD)
        lift!!.setDirection(DcMotorSimple.Direction.FORWARD)

        // Set all motors to zero power
        leftDriveF!!.setPower(0.0)
        rightDriveF!!.setPower(0.0)
        leftDriveB!!.setPower(0.0)
        rightDriveB!!.setPower(0.0)
        leftCollector!!.setPower(0.0)
        tape!!.setPower(0.0)
        lift!!.setPower(0.0)

        // Set all servos to default positions
        openClaw()
        openFoundation()
        close4Bar()
        markerDumper!!.position=(HELD)
        leftSideClaw!!.position=(RELEASEDL)
        rightSideClaw!!.position=(RELEASEDR)
        leftFinger!!.position=(FINGERRELEASEL)
        rightFinger!!.position=(FINGERRELEASER)

        // Set motors to use brake mode
        leftDriveF!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rightDriveF!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        leftDriveB!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rightDriveB!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        tape!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        lift!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        // Use coast on these
        leftCollector!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        rightCollector!!.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

        // Stop and reset the encoders on the necessary motors
        tape!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        lift!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)

        // Set drive motors to run with encoders
        leftDriveF!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        rightDriveF!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        leftDriveB!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        rightDriveB!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        // Odometry encoders on these
        leftCollector!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        leftCollector!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        tape!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        tape!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        leftDriveB!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        leftDriveB!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        // Lift uses encoder
        lift!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER)

        // PIDF
        //leftDriveF.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        //rightDriveF.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        //leftDriveB.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        //rightDriveB.setVelocityPIDFCoefficients(10, 3, 0, 0);//13.65291667);
        //leftDriveF.setPositionPIDFCoefficients(10);
        //rightDriveF.setPositionPIDFCoefficients(10);
        //leftDriveB.setPositionPIDFCoefficients(10);
        //rightDriveB.setPositionPIDFCoefficients(10);


    }


    /**
     * FUNCTIONS
     */

    /**
     * driveSetPower sets all of the drive train motors to the specified power levels.
     * @param powerLF Power level to set front left motor to
     * @param powerRF Power level to set front right motor to
     * @param powerLB Power level to set back left motor to
     * @param powerRB Power level to set back right motor to
     */
    fun driveSetPower (powerLF:Double, powerLB:Double, powerRF:Double, powerRB:Double) {
        leftDriveF!!.power = powerLF
        rightDriveF!!.setPower(powerRF)
        leftDriveB!!.setPower(powerLB)
        rightDriveB!!.setPower(powerRB)
    }

    fun setX (x:Double) {
        globalRobot.setX(x);
    }

    fun setY (y:Double) {
        globalRobot.setY(y);
    }

    fun setAngle (angle:Double) {
        globalRobot.setAngle(angle);
    }

    fun setGlobalRobot (x:Double, y:Double, angle:Double) {
        setX(x)
        setY(y)
        setAngle(angle)
    }

    fun getX():Double {
        return globalRobot.getX()
    }

    fun getY():Double {
        return globalRobot.getY()
    }

    fun getWorldAngle_rad():Double {
        return globalRobot.getAngle()
    }

    fun closeFoundation() {
        foundation!!.setPosition(1.0)
    }

    fun openFoundation() {
        foundation!!.setPosition(0.35)
    }

    fun runCollector(power:Double) {
        leftCollector!!.setPower(power)
        rightCollector!!.setPower(power)
    }

    /**
     * Turns on the collector.
     */
    fun collectorOn() {
        runCollector(-1.0)
    }

    /**
     * Turns off the collector.
     */
    fun collectorOff() {
        runCollector(0.0)
    }

    /**
     * Reverses the collector.
     */
    fun collectorRev () {
        runCollector(1.0)
    }

    /**
     * Sets the position of the 4 bar.
     * @param position The desired position for the left servo. The right one then mirrors it.
     */
    fun set4Bar(position:Double) {
        left4Bar!!.setPosition(position)
        right4Bar!!.setPosition(1 - position)
    }

    /**
     * Swings the 4 bar in to grab a stone
     */
    fun close4Bar () {
        set4Bar(0.0)
    }

    /**
     * Swings the 4 bar out to place a stone.
     */
    fun place4Bar () {
        set4Bar(1.0)
    }

    /**
     * Opens the claw to place a stone.
     */
    fun openClaw() {
        claw!!.setPosition(.35)
    }

    /**
     * Closes the claw to pick up a stone.
     */
    fun closeClaw() {
        claw!!.setPosition(1.0)
    }

    /**
     * driveSetMode sets all of the drive train motors to the specified mode.
     * @param runmode RunMode to set motors to
     */
    fun driveSetMode(runmode:DcMotor.RunMode) {
        leftDriveF!!.setMode(runmode)
        rightDriveF!!.setMode(runmode)
        leftDriveB!!.setMode(runmode)
        rightDriveB!!.setMode(runmode)
    }

    /**
     * Sets the speed of the four drive motors given desired speeds in the robot's x, y, and angle.
     * @param vX Robot speed in the x (sideways) direction.
     * @param vY Robot speed in the y (forwards) direction.
     * @param vA Robot speed in the angle (turning) direction.
     * @param minPower Minimum speed allowed for the average of the four motors.
     * @param maxPower Maximum speed allowed for the fasted of the four motors.
     */
    fun setSpeedAll (vX:Double, vY:Double, vA:Double, minPower:Double, maxPower:Double) {
        // Calculate theoretical values for motor powers using transformation matrix
        var fl = vY + vX - vA
        var bl = vY - vX - vA
        var br = vY + vX + vA
        var fr = vY - vX + vA
        // Find the largest magnitude of power and the average magnitude of power to scale down to
        // maxpower and up to minpower
        var max = Math.abs(fl)
        max = max.coerceAtLeast(abs(bl))
        max = max.coerceAtLeast(abs(br))
        max = max.coerceAtLeast(abs(fr))
        var ave = (abs(fl) + abs(bl) + abs(br) + abs(fr)) / 4
        if (max > maxPower) {
            fl *= maxPower
            bl *= maxPower
            br *= maxPower
            fr *= maxPower
            fl /= max + 1E-6
            bl /= max + 1E-6
            br /= max + 1E-6
            fr /= max + 1E-6
        }
        else if (ave < minPower) {
            fl *= minPower
            bl *= minPower
            br *= minPower
            fr *= minPower
            fl /= max + 1E-6
            bl /= max + 1E-6
            br /= max + 1E-6
            fr /= max + 1E-6
        }
        // Recalculate max and scale down to 1
        max = abs(fl)
        max = max.coerceAtLeast(abs(bl))
        max = max.coerceAtLeast(abs(br))
        max = max.coerceAtLeast(abs(fr))
        if (max < 1)
            max = 1.0
        fl /= max + 1E-6
        bl /= max + 1E-6
        br /= max + 1E-6
        fr /= max + 1E-6
        // Range clip just to be safe
        fl = Range.clip(fl, -1.0, 1.0)
        bl = Range.clip(bl, -1.0, 1.0)
        br = Range.clip(br, -1.0, 1.0)
        fr = Range.clip(fr, -1.0, 1.0)
        // Set powers
        leftDriveB?.setPower(bl)
        leftDriveF?.setPower(fl)
        rightDriveF?.setPower(fr)
        rightDriveB?.setPower(br)
    }

    /**
     * Sets the speeds of the motors to 0.
     */
    fun setSpeedZero () {
        setSpeedAll(0.0, 0.0, 0.0, 0.0, 0.0)
    }

    /**
     * Updates the current position (globalRobot) of the robot based off of the change in the
     * odometry encoders.
     */
    fun updatePosition () {
        var bulkData = expansionHub.getBulkInputData()
        var currentL:Double= -bulkData.getMotorCurrentPosition(lOWheel) / 1144.0
        var currentC:Double = bulkData.getMotorCurrentPosition(cOWheel) / 1144.0
        var currentR:Double = -bulkData.getMotorCurrentPosition(rOWheel) / 1144.0
        deltaL = currentL - previousL
        deltaC = currentC - previousC
        deltaR = currentR - previousR
        previousL = currentL
        previousC = currentC
        previousR = currentR

        globalRobot.updatePosition(deltaL, deltaC, deltaR)
    }
}