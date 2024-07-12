///* Copyright (c) 2019 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import android.hardware.Camera;
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import java.util.*;
//
//import java.util.List;
//
///*
// * This OpMode illustrates the basics of TensorFlow Object Detection,
// * including Java Builder structures for specifying Vision parameters.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
// */
//@Autonomous(name = "encoder test")
////@Disabled
//        robot.setMotors(0,0,0,0);
//        //strafe right ++--
public class encodertest extends LinearOpMode {
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

    @Override
    public void runOpMode() {
        initialize();
        //initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Press Play to start autonomous");
        telemetry.update();
        waitForStart();

        boolean found = false;
        int choose = 0;
        //back ++++
      /* robot.setMotors(0.5,0.5, 0.5, 0.5);
       sleep(500);
       robot.setMotors(0,0,0,0);
        //forward ----
       robot.setMotors(-0.5, -0.5, -0.5, -0.5);
       sleep(500);
       robot.setMotors(0,0,0,0);
       //strafe left --++
        robot.setMotors(-0.5, -0.5, 0.5, 0.5);
        sleep(500);
        robot.setMotors(0.5, 0.5, -0.5, -0.5);
        sleep(500);
        robot.setMotors(0,0,0,0);

        //rotate left -++-
        robot.setMotors(-0.5, 0.5, 0.5, -0.5);
        sleep(500);
        robot.setMotors(0,0,0,0);
        //rotate right +--+
        robot.setMotors(0.5, -0.5, -0.5, 0.5);
        sleep(500);
        robot.setMotors(0,0,0,0);
*/

        robot.timeDrive("F", 0.5, 500);

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */


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