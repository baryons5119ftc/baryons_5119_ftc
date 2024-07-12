package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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