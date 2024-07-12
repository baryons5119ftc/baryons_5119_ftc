package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class RealRobot extends MecanumDrive{

    static final double     COUNTS_PER_MOTOR_REV    = 145.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference || Previous value of 3.93701
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public DcMotorEx lf, lr, rf, rr;
//    public final DcMotor hookSp, hook;
//    public final DcMotorEx intake;
//    public final DcMotor lift;
//    public final Servo dropper;
//    public final Servo launcher;
    public ElapsedTime elapsed = new ElapsedTime();

    //public final Servo grabber,track, trayL, trayR;

    private final BHI260IMU imu;

    private double headingOffset = 0.0;
    private Orientation angles;
    private Acceleration gravity;



    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    static final int MOTOR_TICK_COUNTS = 145;


    public double shooterPos = .56;
    public boolean shooterReady = true;

    //roadrunner variables
     public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    private List<DcMotorEx> motors = Arrays.asList(lf, lr, rr, rf);

    public RealRobot(HardwareMap hardwareMap, final Telemetry _telemetry) {

        //roadrunner stuff
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        //idk if this is needed
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
//        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
//
//        trajectorySequenceRunner = new TrajectorySequenceRunner(
//                follower, HEADING_PID, batteryVoltageSensor,
//                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
//        );



        //hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        /*lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");*/
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
//        hook = hardwareMap.dcMotor.get("hook");
//        hookSp = hardwareMap.dcMotor.get("hookSp");
//        dropper = hardwareMap.servo.get("dropper") ;
//        intake = hardwareMap.dcMotor.get("intake");
//        lift = hardwareMap.dcMotor.get("lift");
//        launcher = hardwareMap.servo.get("launcher");



        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
        //reverses the motors



        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, lf, lr, rf, rr);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, rf, rr, lr);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //initally BRAKE



        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new BHI260IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu.initialize(parameters);
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //zeroPosition = slide.getCurrentPosition();
    }
//start of rr function

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        //need to change to make motors go at vel
        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        lf.setPower(v);
        lr.setPower(v1);
        rr.setPower(v2);
        rf.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

//end of rr for now

    public void setMotorMode(DcMotor.RunMode mode, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(mode);
        }
    }

    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    public void runWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
    }

    /**
     * @return true if the gyro is fully calibrated, false otherwise
     */
//    public boolean isGyroCalibrated() {
//        return imu.isGyroCalibrated();
//    }

    /**
     * Fetch all once-per-time-slice values.
     * <p>
     * Call this either in your OpMode::loop function or in your while(opModeIsActive())
     * loops in your autonomous. It refresh gyro and other values that are computationally
     * expensive.
     */
    public void loop() {
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        gravity = imu.getGravity();
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * @return the robot's current heading in degrees
     */
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _lf Left front motor
     * @param _lr Left rear motor
     * @param _rf Right front motor
     * @param _rr Right rear motor
     */

    public void setMotors(double _lf, double _lr, double _rf, double _rr) {
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);
        lf.setPower(_lf / scale);
        lr.setPower(_lr / scale);
        rf.setPower(_rf / scale);
        rr.setPower(_rr / scale);
    }

    public void driveInches(double speed, double inches){
        lf.setPower(speed);
        lr.setPower(speed);
        rf.setPower(speed);
        rr.setPower(speed);

        lf.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        lr.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        rf.setTargetPosition((int) (inches * COUNTS_PER_INCH));
        rr.setTargetPosition((int) (inches * COUNTS_PER_INCH));

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public void stopRobot(){
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }

    public void drive(double speed) {
        stopRobot();
        setMotors(speed, speed, speed, speed); //deleted minus from second

    }

    public void gyroDrive(double x, double y) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = speed * Math.sin(direction + Math.PI / 4.0);
        double rf = speed * Math.cos(direction + Math.PI / 4.0);
        double lr = speed * Math.cos(direction + Math.PI / 4.0);
        double rr = speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);
    }

    public void speedGyroDrive(double x, double y, double robotSpeed) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, robotSpeed);

        double lf = speed * Math.sin(direction + Math.PI / 4.0);
        double rf = speed * Math.cos(direction + Math.PI / 4.0);
        double lr = speed * Math.cos(direction + Math.PI / 4.0);
        double rr = speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);

        telemetry.addData("RF & LR Speed", Math.cos(direction + Math.PI / 4.0));
        telemetry.update();
    }

    public void gyroDriveSlow(double x, double y) {
        //final double rotation = Math.pow(controller.right_stick_x, 3.0)/1.5;
        double direction = Math.atan2(x, y) + (getHeading());
        double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        double lf = .5 * speed * Math.sin(direction + Math.PI / 4.0);
        double rf = .5 * speed * Math.cos(direction + Math.PI / 4.0);
        double lr = .5 * speed * Math.cos(direction + Math.PI / 4.0);
        double rr = .5 * speed * Math.sin(direction + Math.PI / 4.0);

        setMotors(lf, lr, rf, rr);
    }

    public void setUpGyroDrive(int distance){

        lf.setTargetPosition(distance + lf.getCurrentPosition());
        rf.setTargetPosition(distance + rf.getCurrentPosition());
        lr.setTargetPosition(distance + lr.getCurrentPosition());
        rr.setTargetPosition(distance + rr.getCurrentPosition());

    }

    /**
     *
     * @param power // power (Decimal) .0-1.0
     * @param distance // distance (in inches)
     * @param direction // direction (F = forward, B = back, L = Strafe left, R = Strafe Right)
     */

    public void encoderDrive(double power, double distance, char direction) {
        distance *= 3;
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // How many turns do I need the wheels to go [distance] inches?

        // The distance you drive with one turn of the wheel is the circumference of the wheel

        //Re-measure
        double circumference = ((direction == 'F' || direction == 'B') ? Math.PI*WHEEL_DIAMETER_INCHES : 11.4);
        double TICKS_PER_INCH = MOTOR_TICK_COUNTS/circumference;

        int eTarget = (int)(TICKS_PER_INCH*distance);



        ((DcMotorEx)lf).setTargetPositionTolerance(12);
        ((DcMotorEx)rf).setTargetPositionTolerance(12);
        ((DcMotorEx)lr).setTargetPositionTolerance(12);
        ((DcMotorEx)rr).setTargetPositionTolerance(12);

        if(direction == 'R')
        {
            lf.setTargetPosition(-eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(-eTarget + rr.getCurrentPosition());
        }
        else if (direction == 'L')
        {
            lf.setTargetPosition(eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(-eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(-eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget + rr.getCurrentPosition());
        }
        else if (direction == 'B')
        {
            lf.setTargetPosition(eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget + rr.getCurrentPosition());
        }
        else if (direction == 'F')
        {
            lf.setTargetPosition(-eTarget + lf.getCurrentPosition());
            rf.setTargetPosition(-eTarget + rf.getCurrentPosition());
            lr.setTargetPosition(-eTarget + lr.getCurrentPosition());
            rr.setTargetPosition(-eTarget + rr.getCurrentPosition());
        }

        //set the power desired for the motors
        lf.setPower(power*.7*(direction == 'R' || direction == 'F' ? 1.3 : 1));
        rf.setPower(power*.7*(direction == 'R' || direction == 'F' ? 1.3 : 1));
        lr.setPower(power*.7);
        rr.setPower(power*.7);

        // set the motors to RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy())
        {
            loop();
            // make sure to not do anything while the motors are running
            telemetry.addData("Path", "Driving " + distance + " inches");
            /*telemetry.addData("Slide position:",slide.getCurrentPosition());
            telemetry.addData("Slide target:",slide.getTargetPosition());*/
            telemetry.addData("Current position", lf.getCurrentPosition());
            telemetry.addData("Target position", lf.getTargetPosition());
            telemetry.addData("Heading", getHeadingDegrees());

            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     *
     * @param degrees //degrees (-360 to 360) you want to rotate (Positive is clockwise)
     */
    public void encoderRotate(int degrees, double power) {

        // RESETS ENCODERS

        // Circumference of the circle made by the robot (19-inch diameter * pi)
        double rotationLength = 34.5565;

        // Length the wheels would have to travel in order to rotate 1 degree in length (distance / 360)
        double degreeLength = rotationLength/360.0;

        double distance = Math.abs(degrees)*degreeLength;
        // The distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference = (28/14)*3.14*WHEEL_DIAMETER_INCHES;

        double rotationsNeeded = distance/circumference;

        int eTarget = (int)(rotationsNeeded * MOTOR_TICK_COUNTS);

        ((DcMotorEx)lf).setTargetPositionTolerance(12);
        ((DcMotorEx)rf).setTargetPositionTolerance(12);
        ((DcMotorEx)lr).setTargetPositionTolerance(12);
        ((DcMotorEx)rr).setTargetPositionTolerance(12);
        //Set target position
        if(degrees > 0)
        {
            lf.setTargetPosition(eTarget    + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget*-1 + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget    + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget*-1 + rr.getCurrentPosition());
        }
        else if(degrees < 0)
        {
            lf.setTargetPosition(eTarget*-1 + lf.getCurrentPosition());
            rf.setTargetPosition(eTarget    + rf.getCurrentPosition());
            lr.setTargetPosition(eTarget*-1 + lr.getCurrentPosition());
            rr.setTargetPosition(eTarget    + rr.getCurrentPosition());
        }

        //set the power desired for the motors
        lf.setPower(power*-1);//power used to be *-0.7
        rf.setPower(power*-1);
        lr.setPower(power*-1);
        rr.setPower(power*-1);

//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the motors to RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        while(lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy())
//        {
//            loop();
//            // make sure to not do anything while the motors are running
//            telemetry.addData("Path", "Driving " + distance + " inches");
//            telemetry.addData("Heading", getHeadingDegrees());
//            telemetry.update();
//        }



//        lf.setPower(0);
//        rf.setPower(0);
//        lr.setPower(0);
//        rr.setPower(0);

        while(lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy())
        {
            loop();
            // make sure to not do anything while the motors are running
            telemetry.addData("Path", "Driving " + distance + " inches");
            /*telemetry.addData("Slide position:",slide.getCurrentPosition());
            telemetry.addData("Slide target:",slide.getTargetPosition());*/
            telemetry.addData("Current position", lf.getCurrentPosition());
            telemetry.addData("Target position", lf.getTargetPosition());
            telemetry.addData("Heading", getHeadingDegrees());



            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        //telemetry.addData("Heading: ", getHeadingDegrees());
        telemetry.update();
    }

    public void rotateToHeading(int degrees, double power){
        int head = (int)getHeadingDegrees();
        int diff=head-degrees;
        encoderRotate(-diff, power);
        loop();
        telemetry.addData("Heading: ", getHeadingDegrees());
        while(!(head>degrees-3&&head<degrees+3)){
            encoderRotate(-10, power);
            head = (int)getHeadingDegrees();
            loop();
            telemetry.addData("Heading: ", getHeadingDegrees());
        }
    }

    public void mecanumEncoders()
    {
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sleep(int millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /**
     @param degrees heading that you are rotating to
     */
    public void rotate(double degrees, double power)
    {
        //double endHeading = getHeadingDegrees()+degrees;
        double endHeading = degrees - getHeadingDegrees();
        if(endHeading<-180)
            endHeading+=360;
        else if(endHeading>180)
            endHeading-=360;
        if(degrees > 0)
        {
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            lf.setPower(-power);
            lr.setPower(-power);
            rf.setPower(power);
            rr.setPower(power);
            do {
                loop();
                telemetry.addData("Heading", getHeadingDegrees());
                telemetry.addData("Absolute", Math.abs(degrees-getHeadingDegrees()));
                telemetry.addData("Degrees", degrees);
                telemetry.update();

            } while(Math.abs(getHeadingDegrees()-endHeading) > 4);
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }
        else if(degrees < 0)
        {
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            lf.setPower(power);
            lr.setPower(power);
            rf.setPower(-power);
            rr.setPower(-power);
            do {
                loop();
                telemetry.addData("Heading", getHeadingDegrees());
                telemetry.addData("Absolute", Math.abs(degrees-getHeadingDegrees()));
                telemetry.addData("Degrees", degrees);
                telemetry.update();

            }while(Math.abs(getHeadingDegrees()-endHeading) > 4);
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }
    }

    public double convertHeading(double degrees) {
        if(degrees < -180) return degrees + 360;
        else if(degrees > 180) return degrees - 360;
        else return degrees;
    }

    public void rotateTo(double degrees, double power)
    {
        double currHeading = convertHeading(getHeadingDegrees());
        //double endHeading = getHeadingDegrees()+degrees;
        double endHeading = convertHeading(degrees);

        double diff = convertHeading(endHeading - currHeading);
        if(diff > 0)
        {
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lf.setPower(0.25);
            lr.setPower(0.25);
            rf.setPower(-0.25);
            rr.setPower(-0.25);
            do {
                loop();

                double newPow = Math.abs(currHeading-convertHeading(degrees)) / 300.0;
                lf.setPower(newPow);
                lr.setPower(newPow);
                rf.setPower(-newPow);
                rr.setPower(-newPow);
                currHeading = convertHeading(getHeadingDegrees());
                telemetry.addData("Heading", getHeadingDegrees());
                telemetry.addData("Absolute", Math.abs(degrees));
                telemetry.addData("Degrees", diff);
                telemetry.update();

            } while(Math.abs(currHeading-convertHeading(degrees)) > 4);
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }
        else if(diff < 0)
        {
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lf.setPower(-0.25);
            lr.setPower(-0.25);
            rf.setPower(0.25);
            rr.setPower(0.25);
            do {
                loop();

                double newPow = Math.abs(currHeading-convertHeading(degrees)) / 400.0;
                lf.setPower(-newPow);
                lr.setPower(-newPow);
                rf.setPower(newPow);
                rr.setPower(newPow);
                currHeading = convertHeading(getHeadingDegrees());
                telemetry.addData("Heading", getHeadingDegrees());
                telemetry.addData("Absolute", Math.abs(degrees));
                telemetry.addData("Degrees", diff);
                telemetry.update();

            }while(Math.abs(currHeading-convertHeading(degrees)) > 4);
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }
    }

    public void useEncoders()
    {
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void timeRotate(double heading, String direction, double power){
        if (direction == "CW") {
            while(Math.abs(getHeadingDegrees()-heading)>2){
                setMotors(power, -power, -power, power);
            }
        }
        else{
            while(Math.abs(getHeadingDegrees()-heading)>2){
                setMotors(-power, power, power, -power);
            }
        }
        setMotors(0,0,0,0);
    }

    //outtake area is forward
    public void timeDrive(String direction, double p, int mil){
        if(direction=="F"){
            setMotors(-p,-p,-p,-p);
            sleep(mil);
            setMotors(0,0,0,0);
        }
        else if(direction=="B"){
            setMotors(p,p,p,p);
            sleep(mil);
            setMotors(0,0,0,0);
        }
        else if(direction=="L"){
            setMotors(-p,-p,p,p);
            sleep(mil);
            setMotors(0,0,0,0);
        }
        else if(direction=="R"){
            setMotors(p,p,-p,-p);
            sleep(mil);
            setMotors(0,0,0,0);
        }
    }


    public boolean approxServo(double actualPos, double supposedPos) {
        return (actualPos < supposedPos+0.001 && actualPos > supposedPos-0.001);
    }

    public boolean approxMotor(int actualPos, double supposedPos) {
        return (actualPos < supposedPos + 5 && actualPos > supposedPos - 5);
    }




}

