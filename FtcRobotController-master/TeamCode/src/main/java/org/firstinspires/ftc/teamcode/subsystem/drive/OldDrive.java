package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OldDrive {
    static final double RAW_TICKS_PER_ROTATION = 28;
    static final double MAX_MOTOR_RPM = 6000;
    static final double GEAR_RATIO = 1.0/18.9; //output divided by input
    static final double WHEEL_DIAMETER = 3.0; //inches

    //Calculated Constants
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double GEARED_TICKS_PER_ROTATION = RAW_TICKS_PER_ROTATION / GEAR_RATIO;
    static final double GEARED_MOTOR_RPM = MAX_MOTOR_RPM * GEAR_RATIO;
    static final double INCHES_PER_TICK =  WHEEL_CIRCUMFERENCE / GEARED_TICKS_PER_ROTATION;
    static final double MAX_SPEED = (9.0/12.0) * WHEEL_CIRCUMFERENCE * (GEARED_MOTOR_RPM / 60.0);


    /* -------Variables------- */

    //Hardware
    public IMU imu;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    LinearOpMode opMode;


    //Misc Variables
    public double botHeading;

    /* -------Constructor------- */

    public OldDrive (HardwareMap hardwareMap, LinearOpMode linearopmode) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode = linearopmode;
    }

    /* -------Telemetry------- */

    //Updates telemetry, sends data to driver station



    /* -------Conversions------- */

    //Converts inches to motor rotation ticks
    public static int toTicks(double inches) {
        return (int) Math.round(inches / INCHES_PER_TICK);
    }

    //Converts distance to time based on the velocity of the robot
    public static double distToTime(double inches, double power) {
        return Math.round(((inches - 2) / (MAX_SPEED * power)) * 100) / 100.0;
    }

    /* -------Update Heading------- */

    //Updates heading in degrees
    public void updateHeadingDeg() {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    //Updates heading in radians
    public void updateHeadingRad() {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /* -------Bulk Motor State Functions------- */

    //Set all motor modes
    public void setModes(DcMotor.RunMode mode) {
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
    }

    //Set all motor powers
    public void setPowers(float speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }

    //Set motor targets to their variables
    public void setMotorTargets(double blPos, double brPos, double flPos, double frPos) {
        backLeftMotor.setTargetPosition((int) blPos);
        backRightMotor.setTargetPosition((int) brPos);
        frontLeftMotor.setTargetPosition((int) flPos);
        frontRightMotor.setTargetPosition((int) frPos);
    }

    //Sets all motor powers for complex movement
    public void driveBot(double forward, double strafe, double rotate) {
        frontLeftMotor.setPower((forward + strafe + rotate));
        backLeftMotor.setPower((forward - strafe + rotate));
        frontRightMotor.setPower((forward + strafe - rotate));
        backRightMotor.setPower((forward - strafe - rotate));
    }

    /* -------Autonomous Encoder Based Drive------- */

    //Move forward/backward, in ticks
    public void drive(int leftMove, int rightMove, float speed) {

        double blPos = backLeftMotor.getCurrentPosition() + leftMove;
        double brPos = backRightMotor.getCurrentPosition() + rightMove;
        double flPos = frontLeftMotor.getCurrentPosition() + leftMove;
        double frPos = frontRightMotor.getCurrentPosition() + rightMove;

        setMotorTargets(blPos, brPos, flPos, frPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        while (isBusy(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor)){
            opMode.idle();
        }
    }

    //Strafe left/right, in ticks
    public void strafe(int move, float speed) {

        double blPos = backLeftMotor.getCurrentPosition() - move;
        double brPos = backRightMotor.getCurrentPosition() - move;
        double flPos = frontLeftMotor.getCurrentPosition() + move;
        double frPos = frontRightMotor.getCurrentPosition() + move;

        setMotorTargets(blPos, brPos, flPos, frPos);

        setModes(DcMotor.RunMode.RUN_TO_POSITION);

        setPowers(speed);
        while (isBusy(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor)) {
            opMode.idle();
        }
    }

    //Drives using inches instead of ticks
    public void driveInches(double leftInches, double rightInches, float speed) {
        drive(toTicks(leftInches), toTicks(rightInches), speed);
    }

    //Strafes using inches instead of ticks
    public void strafeInches(double inches, float speed) {
        strafe(toTicks(inches), speed);
    }

    /* -------Autonomous Gyro/Time Based Drive------- */

    //Turn using Gyro, heading is absolute
    public void turnToAbs(double target, double power) {
        updateHeadingDeg();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        while ((botHeading < target - 2 || botHeading > target + 2)) {
            driveBot(0, 0, power);
            updateHeadingDeg();
        }
        setPowers(0);
    }

    //Turn using Gyro, heading is based on robot's initial facing
    public void turnToRel(double relTarg, double power) {
        updateHeadingDeg();
        double target = botHeading + relTarg;
        if (target > 180) {
            target -= 360;
        }
        if (target < -180) {
            target += 360;
        }
        turnToAbs(target, power);
    }

    //Complex time based gyro movement- movement is absolute
    public void rotateAndMove(double seconds, double rotateTarget, double forwardPower, double strafePower, double rotatePower) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double fieldForward;
        double fieldStrafe;
        updateHeadingDeg();
        double oghead = botHeading;
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        while (timer.time() < seconds && (botHeading < rotateTarget - 5 || botHeading > rotateTarget + 5)) {
            updateHeadingRad();
            fieldForward = strafePower * Math.sin(-botHeading) + forwardPower * Math.cos(-botHeading);
            fieldStrafe = strafePower * Math.cos(-botHeading) - forwardPower * Math.sin(-botHeading);
            updateHeadingDeg();
            opMode.telemetry.addData("rot: ", botHeading);
            opMode.telemetry.update();
            if (oghead - rotateTarget < 0) {
                driveBot(fieldForward, fieldStrafe, -rotatePower);
            } else {
                driveBot(fieldForward, fieldStrafe, rotatePower);
            }

        }
        while (timer.time() < seconds) {

            updateHeadingRad();
            fieldForward = strafePower * Math.sin(-botHeading) + forwardPower * Math.cos(-botHeading);
            fieldStrafe = strafePower * Math.cos(-botHeading) - forwardPower * Math.sin(-botHeading);
            updateHeadingDeg();
            if (botHeading < rotateTarget - 5) {
                driveBot(fieldForward, fieldStrafe, -rotatePower * 0.5);
            } else if (botHeading > rotateTarget + 5) {
                driveBot(fieldForward, fieldStrafe, rotatePower * 0.5);
            } else {
                driveBot(fieldForward, fieldStrafe, 0);
            }
            opMode.telemetry.addData("rot: ", botHeading);
            opMode.telemetry.update();

        }

        while (botHeading < rotateTarget - 5 || botHeading > rotateTarget + 5) {
            if (botHeading <= rotateTarget - 5) {
                driveBot(0, 0, -rotatePower * 0.5);
            } else if (botHeading >= rotateTarget + 5) {
                driveBot(0, 0, rotatePower * 0.5);
            } else {
                driveBot(0, 0, 0);
            }
            updateHeadingDeg();
        }
        setPowers(0);
    }

    //Complex gyro based movement in terms of distance instead of time- Also has added functionality for easy diagonal movement (AT LEAST ONE OF forwardDist OR strafeDist MUST BE != 0)
    public void rotateAndMoveInches(double rotateTarget, double forwardDist, double strafeDist, double maxMovePower, double rotatePower) {
        double maxDist = Math.max(Math.abs(forwardDist), Math.abs(strafeDist));
        double forwardPower;
        double strafePower;
        double travelTime;
        if (maxDist == Math.abs(forwardDist)) {
            forwardPower = Math.abs(maxMovePower);
            if (forwardDist < 0) {
                forwardPower *= -1;
            }
            strafePower = forwardPower * (strafeDist / forwardDist);
        } else {
            strafePower = Math.abs(maxMovePower);
            if (strafeDist < 0) {
                strafePower *= -1;
            }
            forwardPower = strafePower * (forwardDist / strafeDist);
        }
        travelTime = distToTime(maxDist, maxMovePower);
        rotateAndMove(travelTime, rotateTarget, forwardPower, strafePower, rotatePower);
    }

    //Idle function
    public boolean isBusy(DcMotor... motors) {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }



}
