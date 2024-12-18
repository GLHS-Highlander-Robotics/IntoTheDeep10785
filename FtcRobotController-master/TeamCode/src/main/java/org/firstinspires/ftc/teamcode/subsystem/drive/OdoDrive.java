package org.firstinspires.ftc.teamcode.subsystem.drive;

import org.firstinspires.ftc.teamcode.subsystem.PID;
import org.firstinspires.ftc.teamcode.subsystem.math.vec2d;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OdoDrive {
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


    //PID Coefficients
    static double p,i,d;


    /* -------Variables------- */

    //Hardware
    public IMU imu;
    public SparkFunOTOS otos;
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    LinearOpMode opMode;
    ElapsedTime passed_time;

    /**
      Constructor:
     **/
    public OdoDrive(HardwareMap hardwareMap, LinearOpMode linearopmode) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor_front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor_front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor_back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor_back_right");
        otos = hardwareMap.get(SparkFunOTOS.class, "my_otos");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode = linearopmode;
    }

    //Set all motor modes
    public void setModes(DcMotor.RunMode mode) {
        backLeftMotor.setMode(mode);
        backRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
    }

    //Set all motor powers
    public void setPowers(double speedlf, double speedrf, double speedlb, double speedrb) {
        backLeftMotor.setPower(speedlb);
        backRightMotor.setPower(speedrb);
        frontLeftMotor.setPower(speedlf);
        frontRightMotor.setPower(speedrf);
    }

    //Set motor targets to their variables
    public void setMotorTargets(double blPos, double brPos, double flPos, double frPos) {
        backLeftMotor.setTargetPosition((int) blPos);
        backRightMotor.setTargetPosition((int) brPos);
        frontLeftMotor.setTargetPosition((int) flPos);
        frontRightMotor.setTargetPosition((int) frPos);
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

    public void dir_to_drive(vec2d movement){
        double x = Math.sin(Math.toRadians(movement.dir));
        double y = Math.cos(Math.toRadians(movement.dir));

        x=applySymmetricDeadband(x,0.05);
        y=applySymmetricDeadband(y,0.05);

        // SET POWERS OR ENCODERS?
        setPowers(y+x, y-x, y-x, y+x);
    }

    double PIDrive;

    double clamp(double value, double minimum, double maximum){
        if(value<minimum){
            return minimum;
        }
        else if(value>maximum){
            return maximum;
        }
        else return value;
    }



    public void driveTo(double x, double y, double h, double time){

    }

    public void PIDrive(double x_t, double y_t, double h_t, double maxTime){
        double x_pow=0, y_pow=0, h_pow=0, max;
        double lf=0, rf=0, lb=0, rb=0;

        PID x = new PID(x_t);
        PID y = new PID(y_t);
        PID h = new PID(h_t);
        passed_time.reset();

        while((passed_time.milliseconds()<=0.01)||(x.applySymmetricDeadband(x.error(otos.getPosition().x), 0.1)!=0 || y.applySymmetricDeadband(y.error(otos.getPosition().y), 0.1)!=0 || h.applySymmetricDeadband(h.error(imu.getRobotYawPitchRollAngles().getYaw()), 0.1)!=0)){
            x_pow = x.calculate(otos.getPosition().x, passed_time.milliseconds());
            y_pow = y.calculate(otos.getPosition().y, passed_time.milliseconds());
            h_pow = h.calculate(otos.getPosition().h, passed_time.milliseconds());

            lb=y_pow-x_pow+h_pow;
            lf=y_pow+x_pow+h_pow;
            rb=y_pow+x_pow-h_pow;
            rf=y_pow-x_pow-h_pow;

            frontLeftMotor.setPower(lf);
            backLeftMotor.setPower(lb);
            frontRightMotor.setPower(rf);
            backRightMotor.setPower(rb);
        }
        x.reset();
        y.reset();
        h.reset();
    }

}
