package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RealRobot;

@Autonomous(name = "testing1")
public class testing1 extends OpMode {
    RealRobot robot;

    @Override
    public void init() {
        telemetry.addData("Status", "Initiated");
    }

    @Override
    public void loop(){
        telemetry.addData("Status","Looping");
    }
}