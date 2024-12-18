/* Copyright (c) 2021 FIRST. All rights reserved.
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
package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.subsystem.drive.OdoDrive;
import org.firstinspires.ftc.teamcode.subsystem.math.vec2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.slide.LinearSlide;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto PID", group="auto")
//@Disabled
public class auto_odo_encoder extends LinearOpMode {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.03;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.03;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)
    double SPEED_CAP = 0.375; // Between 0 and 1, idk what default should be
    double TURN_CAP = 0.375; // Between 0 and 1, idk what default should be
    double ovr_H_err = 0;

    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members for each of the 4 drive motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    LinearSlide slide;
    OdoDrive drive;

    // Sensors
    private SparkFunOTOS myOtos;        // Optical tracking odometry sensor
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightBack");


        slide = new LinearSlide(hardwareMap);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "my_otos");
        //**INITIALIZATION**

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        //Wait 1 second
        sleep(1000);

        //On initialization
        while (!isStarted()) {
            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("X", 0);
            telemetry.addData("Y", 0);
            telemetry.addData("H", 0);
            telemetry.addData("H_err", 0);
            telemetry.addData("H_Targ", 0);
            telemetry.addData("LF", 0);
            telemetry.addData("RF", 0);
            telemetry.addData("LB", 0);
            telemetry.addData("RB", 0);
            telemetry.addData("Delta y", 0);
            telemetry.update();
            slide.setArmPos(0, 0); //set arm to default state
            slide.ungrabAll(); //
            slide.rotServo.setPosition(0); //wrist down

        }
        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.update();

        forward();
    }

    private void forward(){
        drive.PIDrive(12,0,0,5);
    }
    private void backward(){
        otosDrive(0,-12,0,2);
    }
    private void strafe(){
        otosDrive(12,0,0,2);
    }
    private void forward_strafe(){
        otosDrive(12,0,0,2);
        stopBot();
        otosDrive(12,12,0,2);
        stopBot();
    }
    private void servo(){
        slide.grabAll();
        sleep(1000);
        slide.ungrabAll();
        sleep(1000);
    }
    private void diag(){
        otosDrive(12,12,0,2);
    }
    private void forwardClaw(){
        otosDrive(0,12,0,2);
        stopBot();
        slide.setRot(570);
        sleep(1000);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of`
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(7, 0.25, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.027);
        myOtos.setAngularScalar(0.997);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0,0,0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }


    private SparkFunOTOS.Pose2D errorPos(double x, double y, double h){
        return new SparkFunOTOS.Pose2D(x-myOtos.getPosition().x, y-myOtos.getPosition().y, h-myOtos.getPosition().h);
    }

    public static double applySymmetricDeadband(double input, double deadbandThreshold) {
        // If the absolute value of input is within the deadband, return 0
        if (Math.abs(input) <= deadbandThreshold) {
            return 0.0;
        }
        else if (1-Math.abs(input)<=deadbandThreshold){
            return (Math.signum(input));
        }
        // Determine the sign and magnitude outside the deadband
        return input;
    }
    /**
     * Move robot to a designated X,Y position and heading
     * set the maxTime to have the driving logic timeout after a number of seconds.
     */
    void otosDrive(double targetX, double targetY, double targetH, double maxTime) {
        double errX, errY, errH;
        vec2d moveInstant;
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0,0,0);
        myOtos.setPosition(currentPosition);
        //Calculate error
        errX=errorPos(targetX, targetY, targetH).x;
        errY=errorPos(targetX, targetY, targetH).y;
        errH=errorPos(targetX, targetY, targetH).h;

        runtime.reset();
        while((opModeIsActive())&&((runtime.milliseconds()<=750*maxTime)&&(Math.abs(errX) >=1 || Math.abs(errY) >= 1))){
            moveInstant = new vec2d(errX, errY, 0);
            driveBot(moveInstant.dir);//Calculate error
            errX=errorPos(targetX, targetY, targetH).x;
            errY=errorPos(targetX, targetY, targetH).y;
            errH=errorPos(targetX, targetY, targetH).h;
            telemetry.addData("H_err", errH);
            telemetry.addData("H_Targ", targetH);
        }

        runtime.reset();
        while((opModeIsActive())&&((runtime.milliseconds()<=750*maxTime)&&(Math.abs(errH) >=13))){
            if(targetH==0){
                break;
            }
            moveInstant = new vec2d(errX, errY, 0);
            rotateBot((errH)/Math.abs(errH));//Calculate error
            errX=errorPos(targetX, targetY, targetH).x;
            errY=errorPos(targetX, targetY, targetH).y;
            errH=errorPos(targetX, targetY, targetH).h;
            telemetry.addData("H_err", errH);
            telemetry.addData("H_Targ", targetH);
        }
        ovr_H_err += errH;
    }

    double clamp(double value, double minimum, double maximum){
        if(value<minimum){
            return minimum;
        }
        else if(value>maximum){
            return maximum;
        }
        else return value;
    }

    void driveBot(double angle_deg){
        double angle_rad = Math.toRadians(angle_deg);
        double y = Math.cos(angle_rad);
        double x = Math.sin(angle_rad);
        y= applySymmetricDeadband(y,0.1);
        x= applySymmetricDeadband(x,0.1);
        double d1 = y+x;
        double d2 = y-x;
        double max = Math.max(Math.abs(d1),Math.abs(d2));
        d1/=max;
        d2/=max;
        d1*=SPEED_CAP;
        d2*=SPEED_CAP;
        d1 = clamp(Math.abs(d1), 0, SPEED_CAP)*Math.signum(d1);
        d2 = clamp(Math.abs(d2), 0, SPEED_CAP)*Math.signum(d2);

        leftFrontDrive.setPower(d1);
        rightBackDrive.setPower(d1);
        rightFrontDrive.setPower(d2);
        leftBackDrive.setPower(d2);

        telemetry.addData("X", myOtos.getPosition().x);
        telemetry.addData("Y", myOtos.getPosition().y);
        telemetry.addData("Ydir", y);
        telemetry.addData("Xdir", x);
        telemetry.addData("H", myOtos.getPosition().h);
        telemetry.addData("LF", leftFrontDrive.getPower());
        telemetry.addData("RF", rightFrontDrive.getPower());
        telemetry.addData("LB", leftBackDrive.getPower());
        telemetry.addData("RB", rightBackDrive.getPower());
        telemetry.update();
    }
    void rotateBot(double r){
        leftFrontDrive.setPower(r*TURN_CAP);
        rightBackDrive.setPower(r*TURN_CAP*-1);
        rightFrontDrive.setPower(r*TURN_CAP*-1);
        leftBackDrive.setPower(r*TURN_CAP);
        telemetry.addData("X", myOtos.getPosition().x);
        telemetry.addData("Y", myOtos.getPosition().y);
        telemetry.addData("H", myOtos.getPosition().h);
        telemetry.addData("H_err", 0);
        telemetry.addData("LF", leftFrontDrive.getPower());
        telemetry.addData("RF", rightFrontDrive.getPower());
        telemetry.addData("LB", leftBackDrive.getPower());
        telemetry.addData("RB", rightBackDrive.getPower());
        telemetry.update();
    }

    void stopBot(){
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }
}
