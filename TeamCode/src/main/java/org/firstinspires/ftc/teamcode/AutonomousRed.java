/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.*;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "autonomous RED")
//@Disabled
public class AutonomousRed extends LinearOpMode {
    RealRobot robot;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "red.tflite";
    private static final String LABELS[] = {"red"};

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    //@Override
    //@Disabled
    public void runOpMode() {
        initialize();
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Press Play to start autonomous");
        telemetry.update();
        waitForStart();

        boolean found = false;
        int choose = 0;

        visionPortal.close();

       /* if (choose == 1) {
            // strafe left
            robot.encoderDrive(0.3, 10.0, 'L');
            // move forward
            robot.encoderDrive(0.3, 20.0, 'F');
            // place pixel
          /*  robot.intake.setTargetPosition(robot.intake.getCurrentPosition()+300);
            robot.intake.setPower(0.5);
            robot.sleep(1000);
            robot.intake.setPower(0);*/
            // move back
           // robot.encoderDrive(0.3, 20.0, 'B');
            // strafe right
            //robot.encoderDrive(0.3, 10.0, 'R');
      /*  } else if (choose == 2) {
            // move forward
            robot.encoderDrive(0.3, 20.0, 'F');
            // place pixel
           /* robot.intake.setTargetPosition(robot.intake.getCurrentPosition()+300);
            robot.intake.setPower(0.5);
            robot.sleep(1000);
            robot.intake.setPower(0);*/
            // move back
            //robot.encoderDrive(0.3, 20.0, 'B');
      /*  } else {
            // strafe right
            robot.encoderDrive(0.3, 10.0, 'R');
            // move forward
            robot.encoderDrive(0.3, 20.0, 'F');
            // place pixel
           /* robot.intake.setTargetPosition(robot.intake.getCurrentPosition()+300);
            robot.intake.setPower(0.5);
            robot.sleep(1000);
            robot.intake.setPower(0);*/
            // move back
          //  robot.encoderDrive(0.3, 20.0, 'B');
            // strafe left
          //  robot.encoderDrive(0.3, 10.0, 'L');
        //}

        // turn right
       // robot.rotateToHeading(90, 1.0);
        // move forward
        //robot.encoderDrive(0.3, 30.0, 'F');
        // place pixels

//        robot.lift.setTargetPosition(-827);
//        robot.sleep(2000);
//        robot.dropper.setDirection(Servo.Direction.FORWARD);
//        robot.dropper.setPosition(0.0);
        // dropper 0.5 = closed
        // dropper 0.0 = opens
        // lift -827
        // end of auto
        robot.encoderDrive(0.5, 5, 'B');
        robot.rotateToHeading(90, 0.5);
        robot.encoderDrive(0.5, 50, 'B');
        robot.sleep(20000);
    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName("redfinal1.tflite")
                .setModelAssetName("red.tflite")

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .setMaxNumRecognitions(1)

                //.setModelAssetName("redfinal1.tflite").setModelLabels(LABELS).
                .build();
        //tfod.setZoom(0.9);
        //tfod.setMinResultConfidence((float) 0.);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public void initialize() {
        ArrayList<DcMotor> allMotors = new ArrayList<>();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot = new RealRobot(hardwareMap, telemetry);
        allMotors.add(robot.lf);
        allMotors.add(robot.rf);
        allMotors.add(robot.lr);
        allMotors.add(robot.rr);

        //robot.carriage.setPosition(.77);
        //robot.carriage.setDirection(Servo.Direction.FORWARD);

        for (DcMotor dcMotor : allMotors) {
            dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }   // end class
}