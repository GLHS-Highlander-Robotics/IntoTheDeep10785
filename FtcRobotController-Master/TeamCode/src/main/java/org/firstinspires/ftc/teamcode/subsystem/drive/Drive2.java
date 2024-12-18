package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive2 {

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

    public Drive2(HardwareMap hardwareMap, LinearOpMode linearopmode){
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
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

    public void setMotorPowers(double lf, double rf, double lb, double rb){
        frontLeftMotor.setPower(lf);
        frontRightMotor.setPower(rf);
        backLeftMotor.setPower(lb);
        backRightMotor.setPower(rb);
    }
    public void stop(){
        setMotorPowers(0,0,0,0);
    }

    public void findNearestPoint(){

    }
}
