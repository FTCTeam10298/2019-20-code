package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Disabled
@Autonomous(name= "opencvSkystoneDetector", group="Sky autonomous")
//@Disabled//comment out this line before using
class KOpenCVSkystoneDetector: LinearOpMode() {
    var runtime: ElapsedTime = ElapsedTime()
    companion object {
        //0 means skystone, 1 means yellow stone
        //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
        private var valMid: Double = 0.0
        private var valLeft: Double = 0.0
        private var valRight: Double = 0.0

        var rectHeight: Float = .6f / 8f
        var rectWidth: Float = 1.5f / 8f

        var offsetX: Float = 0f / 8f//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        var offsetY: Float = 0f / 8f//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

        var midPos = arrayOf(4f / 8f + offsetX, 4f / 8f + offsetY) //0 = col, 1 = row
        var leftPos = arrayOf(2f / 8f + offsetX, 4f / 8f + offsetY)
        var rightPos = arrayOf(6f / 8f + offsetX, 4f / 8f + offsetY)
        //moves all rectangles right or left by amount. units are in ratio to monitor
    }

    val rows: Int= 640
    val cols: Int= 480

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        var cameraMonitorViewId: Int = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        var webcam: OpenCvCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        webcam.openCameraDevice()//open camera
        webcam.setPipeline(StageSwitchingPipeline())//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT)//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart()
        runtime.reset()
        while (opModeIsActive()) {
            telemetry.addData("Values", "$valLeft\n$valMid\n$valRight")
            telemetry.addData("Height", rows)
            telemetry.addData("Width", cols)
            if (valLeft < valMid && valLeft < valRight) {
                telemetry.addData("Skystone:", "Left")
            }
            else if (valMid < valLeft && valMid < valRight) {
                telemetry.addData("Skystone:", "Middle")
            }
            else {
                telemetry.addData("Skystone:", "Right")
            }

            telemetry.update()
            sleep(100)
//            call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
    }

//    detection pipeline
    class StageSwitchingPipeline: OpenCvPipeline() {

        var yCbCrChan2Mat: Mat= Mat()
        var thresholdMat: Mat= Mat()
        var all: Mat= Mat()
        var contoursList: MutableList<MatOfPoint> = ArrayList()

        enum class Stage {//color difference. greyscale
            Detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        var stageToRenderToViewport: Stage= Stage.Detection
        var stages: Array<Stage> = Stage.values()

        override fun onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            var currentStageNum: Int= stageToRenderToViewport.ordinal

            var nextStageNum: Int= currentStageNum + 1

            if(nextStageNum >= stages.size) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages.get(nextStageNum)
        }

        override fun processFrame(input: Mat): Mat{
            contoursList.clear()
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb)//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2)//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102.0, 255.0, Imgproc.THRESH_BINARY_INV)

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)
            yCbCrChan2Mat.copyTo(all)//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8)//draws blue contours


            //get values from frame
            //gets value at circle
            // gets value at circle
            // gets value at circle
            var midSum: Double= 0.0
            var leftSum: Double= 0.0
            var rightSum: Double= 0.045
            for (i in (-10..10)) {
                for (j in (-10..10)) {
                    var pixMid = thresholdMat.get((input.rows() * midPos[1]).toInt() + i, (input.cols() * midPos[0]).toInt() + j)
                    var pixLeft = thresholdMat.get((input.rows() * leftPos[1]).toInt() + i, (input.cols() * leftPos[0]).toInt() + j)
                    var pixRight = thresholdMat.get((input.rows() * rightPos[1]).toInt() + i, (input.cols() * rightPos[0]).toInt() + j)
                    midSum += pixMid[0]
                    leftSum += pixLeft[0]
                    rightSum += pixRight[0]
                }
            }

            valMid = midSum / 400
            valLeft = leftSum / 400
            valRight = rightSum / 400

            //create three points
            var pointMid: Point= Point((input.cols()* midPos[0]).toDouble(), (input.rows()* midPos[1]).toDouble())
            var pointLeft: Point= Point((input.cols()* leftPos[0]).toDouble(), (input.rows()* leftPos[1]).toDouble())
            var pointRight: Point= Point((input.cols()* rightPos[0]).toDouble(), (input.rows()* rightPos[1]).toDouble())

            //draw circles on those points
            Imgproc.circle(all, pointMid,10, Scalar( 255.0, 0.0, 0.0 ),1 )//draws circle
            Imgproc.circle(all, pointLeft,10, Scalar( 255.0, 0.0, 0.0 ),1 )//draws circle
            Imgproc.circle(all, pointRight,10, Scalar( 255.0, 0.0, 0.0 ),1 )//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    Point(
                            input.cols()*(leftPos[0])-10.toDouble(),
                            input.rows()*(leftPos[1])-10.toDouble()
                    ),
                    Point(
                            input.cols()*(leftPos[0])+10.toDouble(),
                            input.rows()*(leftPos[1])+10.toDouble()
                    ),
                    Scalar(0.0, 255.0, 255.0),
                    3)

            Imgproc.rectangle(//3-5
                    all,
                    Point(
                            input.cols()*(midPos[0])-10.toDouble(),
                            input.rows()*(midPos[1])-10.toDouble()
                    ),
                    Point(
                            input.cols()*(midPos[0])+10.toDouble(),
                            input.rows()*(midPos[1])+10.toDouble()
                    ),
                    Scalar(0.0, 255.0, 255.0),
                    3)

            Imgproc.rectangle(//5-7
                    all,
                    Point(
                            input.cols()*(rightPos[0])-10.toDouble(),
                            input.rows()*(rightPos[1])-10.toDouble()
                    ),
                    Point(
                            input.cols()*(rightPos[0])+10.toDouble(),
                            input.rows()*(rightPos[1])+10.toDouble()
                    ),
                    Scalar(0.0, 255.0, 255.0),
                    3)

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    Point(
                            input.cols()*(leftPos[0]-rectWidth/2).toDouble(),
                            input.rows()*(leftPos[1]-rectHeight/2).toDouble()
                    ),
                    Point(
                            input.cols()*(leftPos[0]+rectWidth/2).toDouble(),
                            input.rows()*(leftPos[1]+rectHeight/2).toDouble()
                    ),
                    Scalar(0.0, 255.0, 0.0),
                    3)

            Imgproc.rectangle(//3-5
                    all,
                    Point(
                            input.cols()*(midPos[0]-rectWidth/2).toDouble(),
                            input.rows()*(midPos[1]-rectHeight/2).toDouble()
                    ),
                    Point(
                            input.cols()*(midPos[0]+rectWidth/2).toDouble(),
                            input.rows()*(midPos[1]+rectHeight/2).toDouble()
                    ),
                    Scalar(0.0, 255.0, 0.0),
                    3)

            Imgproc.rectangle(//5-7
                    all,
                    Point(
                            input.cols()*(rightPos[0]-rectWidth/2).toDouble(),
                            input.rows()*(rightPos[1]-rectHeight/2).toDouble()
                    ),
                    Point(
                            input.cols()*(rightPos[0]+rectWidth/2).toDouble(),
                            input.rows()*(rightPos[1]+rectHeight/2).toDouble()
                    ),
                    Scalar(0.0, 255.0, 0.0),
                    3)

            return when (stageToRenderToViewport) {
                Stage.THRESHOLD -> thresholdMat

                Stage.Detection -> all

                Stage.RAW_IMAGE -> input
            }

        }
    }
}