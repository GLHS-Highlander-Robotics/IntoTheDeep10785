package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class BetterFixedDrive {
    static final double RAW_TICKS_PER_ROTATION = 28;
    static final double MAX_MOTOR_RPM = 6000;
    static final double GEAR_RATIO = 1.0/18.9; //output divided by input
    static final double WHEEL_DIAMETER = 2.952756; //inches

    //Calculated Constants
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double GEARED_TICKS_PER_ROTATION = RAW_TICKS_PER_ROTATION / GEAR_RATIO;
    static final double GEARED_MOTOR_RPM = MAX_MOTOR_RPM * GEAR_RATIO;
    static final double INCHES_PER_TICK =  WHEEL_CIRCUMFERENCE / GEARED_TICKS_PER_ROTATION;
    static final double LOAD_COMPENSATION = (15.0/24.0)*(13.0/12.0)*(34.0/30.0)*(10.0/12.0)*(26.5/22);
    static final double MAX_SPEED_SECONDS = WHEEL_CIRCUMFERENCE * (GEARED_MOTOR_RPM / 60.0) * LOAD_COMPENSATION;


    /* -------Variables------- */

    //Hardware
    public IMU imu;
    List<LynxModule> allHubs;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    LinearOpMode opMode;


    //Misc Variables
    public double botHeading;

    /* -------Constructor------- */

    public BetterFixedDrive(HardwareMap hardwareMap, LinearOpMode linearopmode) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor_back_right");
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

        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode = linearopmode;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
        return Math.round(((inches) / (MAX_SPEED_SECONDS * power)) * 100) / 100.0;
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
    //Added multiplier so that at fast speeds the powers are proportional
    public void driveBot(double forward, double strafe, double rotate) {

        double proportion = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate) , 1.0);

        frontLeftMotor.setPower((forward + strafe + rotate) / proportion);
        backLeftMotor.setPower((forward - strafe + rotate) / proportion);
        frontRightMotor.setPower((forward + strafe - rotate) / proportion);
        backRightMotor.setPower((forward - strafe - rotate) / proportion);
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

    //Complex time based gyro movement- rotation is absolute, movement is relative. Added proportional rotation which also simplifies sequence
    public void rotateAndMove(double seconds, double rotateTarget, double forwardPower, double strafePower, double rotatePower) {
        double fieldForward;
        double fieldStrafe;
        double realRot;
        updateHeadingDeg();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.time() < seconds) {
            updateHeadingRad();
            fieldForward = strafePower * Math.sin(-botHeading) + forwardPower * Math.cos(-botHeading);
            fieldStrafe = strafePower * Math.cos(-botHeading) - forwardPower * Math.sin(-botHeading);
            if (Math.abs(botHeading - Math.toRadians(rotateTarget)) > (Math.PI/2.0)) {
                if (botHeading - Math.toRadians(rotateTarget) > 0) {
                    realRot = rotatePower;
                } else {
                    realRot = -rotatePower;
                }
            } else {
                realRot = rotatePower * Math.sin((botHeading - Math.toRadians(rotateTarget)) / 2.0);
            }
            updateHeadingDeg();
            opMode.telemetry.addData("rot: ", botHeading);
            opMode.telemetry.update();
            driveBot(fieldForward,fieldStrafe,realRot);
        }

        while (botHeading < rotateTarget - 1 || botHeading > rotateTarget + 1) {
            updateHeadingRad();
            if (Math.abs(botHeading - Math.toRadians(rotateTarget)) > (Math.PI/2.0)) {
                if (botHeading - Math.toRadians(rotateTarget) > 0) {
                    realRot = rotatePower;
                } else {
                    realRot = -rotatePower;
                }
            } else {
                realRot = rotatePower * Math.sin((botHeading - Math.toRadians(rotateTarget)) / 2.0);
            }
            driveBot(0,0,realRot);
            updateHeadingDeg();
            opMode.telemetry.addData("rot: ", botHeading);
            opMode.telemetry.update();
        }
        setPowers(0);
    }

    //Complex gyro based movement in terms of distance instead of time- Also has added functionality for easy diagonal movement
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

    //The most complex movement
    public void advramInches(double rotateTarget, double forwardDist, double strafeDist, double maxMovePower, double rotatePower) {
        double checkLoopTime = 0.1;
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

        double fieldForward;
        double fieldStrafe;
        double realRot;
        double trueStrafe = strafePower;
        double trueForward = forwardPower;
        double distAlrTrav = 0;
        updateHeadingDeg();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runTimer = new ElapsedTime();
        runTimer.reset();
        ElapsedTime checkTimer = new ElapsedTime();
        checkTimer.reset();
        double checkTime = runTimer.time();

        while (distAlrTrav < Math.abs(maxDist)) {
            updateHeadingRad();
            fieldForward = strafePower * Math.sin(-botHeading) + forwardPower * Math.cos(-botHeading);
            fieldStrafe = strafePower * Math.cos(-botHeading) - forwardPower * Math.sin(-botHeading);
            if (Math.abs(botHeading - Math.toRadians(rotateTarget)) > (Math.PI/2.0)) {
                if (botHeading - Math.toRadians(rotateTarget) > 0) {
                    realRot = rotatePower;
                } else {
                    realRot = -rotatePower;
                }
            } else {
                realRot = rotatePower * Math.sin((botHeading - Math.toRadians(rotateTarget)) / 2.0);
            }
            updateHeadingDeg();
            opMode.telemetry.addData("rot: ", botHeading);
            opMode.telemetry.addData("Distance traveled: ", distAlrTrav);
            opMode.telemetry.update();
            driveBot(fieldForward,fieldStrafe,realRot);


            //brokerunner time correction
            if (runTimer.time() - checkTime >= 0.05) {
                double trueFL = frontLeftMotor.getPower();

                double trueBL = backLeftMotor.getPower();
                double trueBR = backRightMotor.getPower();
                double timeInc;

                trueForward = (((trueBL - trueBR) / 2.0) * Math.sin(-botHeading)) + (((trueFL + trueBR) / 2.0) * Math.cos(-botHeading));
                trueStrafe = (((trueBL - trueBR) / 2.0) * Math.cos(-botHeading)) - (((trueFL + trueBR) / 2.0) * Math.sin(-botHeading));

                if (maxDist == Math.abs(forwardDist)) {
                    distAlrTrav += Math.abs(trueForward) * (runTimer.time() - checkTime) * MAX_SPEED_SECONDS;
                    timeInc = distToTime(maxDist - distAlrTrav, trueForward);
                } else {
                    distAlrTrav += Math.abs(trueStrafe) * (runTimer.time() - checkTime) * MAX_SPEED_SECONDS;
                    timeInc = distToTime(maxDist - distAlrTrav, trueStrafe);
                }
                if (timeInc < 0) {
                    timeInc = 0;
                }
                travelTime = runTimer.time() + timeInc;
                checkTime = runTimer.time();
            }

        }

        while (botHeading < rotateTarget - 1 || botHeading > rotateTarget + 1) {
            updateHeadingRad();
                if (botHeading - Math.toRadians(rotateTarget) > 0) {
                    realRot = rotatePower;
                } else {
                    realRot = -rotatePower;
                }

            driveBot(0,0,realRot);
            updateHeadingDeg();
            opMode.telemetry.addData("rot: ", botHeading);
            opMode.telemetry.update();
        }
        setPowers(0);
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
