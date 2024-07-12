package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class mm3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor rr = hardwareMap.dcMotor.get("rr");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lfPower = (y + x + rx) / denominator;
            double lrPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rrPower = (y + x - rx) / denominator;

            lf.setPower(lfPower);
            lr.setPower(lrPower);
            rf.setPower(rfPower);
            rr.setPower(rrPower);
        }
    }
}