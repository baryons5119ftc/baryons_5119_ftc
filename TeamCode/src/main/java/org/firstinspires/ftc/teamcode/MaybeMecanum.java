
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "MaybeMecanum", group = "Iterative OpMode")
public class MaybeMecanum extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime elapsed = new ElapsedTime();
    private double start;
    //private DcMotor lr = null;
    //private DcMotor rr = null;
    private RealRobot robot;
    private Controller controller;
    private Controller controller2;
    private boolean arcadeMode = false;
    private boolean incrementMode = false;
    private boolean slowMode = false;
    private boolean headingReset = false;
    public static double governor = 0.7;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new RealRobot(hardwareMap, telemetry);

        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        controller.update();
        if (controller.XOnce()) {
            slowMode = !slowMode;
        }
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        elapsed.reset();
        robot.resetHeading();
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.lf, robot.rf, robot.rr, robot.lr);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        controller.update();
        controller2.update();
        robot.loop();

        final double x = Math.pow(controller.left_stick_x * 1, 3.0);
        final double y = Math.pow(controller.left_stick_y * 1, 3.0);

        final double rotation = -Math.pow(controller.right_stick_x * 1, 3.0) / 2.5;
        final double direction = -((Math.atan2(-x, y) + (arcadeMode ? robot.getHeading() : 0.0)));
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = (slowMode ? governor * .6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) + (slowMode ? .3 : 1) * rotation;
        double rf = (slowMode ? governor * .6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) - (slowMode ? .3 : 1) * rotation;
        double lr = (slowMode ? governor * .6 : governor) * speed * Math.sin(direction + Math.PI / 4.0) + (slowMode ? .3 : 1) * rotation;
        double rr = (slowMode ? governor * .6 : governor) * speed * Math.cos(direction + Math.PI / 4.0) - (slowMode ? .3 : 1) * rotation;

        /**
         * Controller button position reference:
         * Top (Triangle): Y
         * Left (Square): X
         * Bottom (X): A
         * Right (Circle): B
         */


        if (controller2.BOnce()) {
            robot.resetHeading();
        }

        if (controller2.YOnce()) {
            arcadeMode = !arcadeMode;
        }


        if (controller2.AOnce()) {
            incrementMode = !incrementMode;
        }

        // Toggles between slowmode
        if (controller.XOnce()) {
            slowMode = !slowMode;
        }

        // Rotates to heading 0
        if (controller.BOnce()) {
            arcadeMode = !arcadeMode;
        }



        robot.setMotors(lf, lr, rf, rr);

        telemetry.addData("Arcade Mode (a)", arcadeMode ? "YES" : "no.");
        telemetry.addData("Slow Mode (s)", slowMode ? "YES" : "no.");
        telemetry.addData("Increment Mode (s)", incrementMode ? "YES" : "no.");
        telemetry.addData("Heading", robot.getHeadingDegrees());
        telemetry.addData("LF Position", robot.lf.getCurrentPosition());
        telemetry.addData("RF Position", robot.rf.getCurrentPosition());
        telemetry.addData("LR Position", robot.lr.getCurrentPosition());
        telemetry.addData("RR Position", robot.rr.getCurrentPosition());
        telemetry.addData("1 Left Joystick Y", controller.left_stick_y);
        telemetry.addData("1 Left Joystick X", controller.left_stick_x);
        telemetry.addData("2 Left Joystick Y", controller2.left_stick_y);
        telemetry.addData("2 Left Joystick X", controller2.left_stick_x);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}