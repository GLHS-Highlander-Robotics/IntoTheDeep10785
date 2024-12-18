package org.firstinspires.ftc.teamcode.subsystem.slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class LinearSlide {

    //Slide powers
    public static double MIN_POWER_SLIDE = 0;
    public static double HOLD_POWER_SLIDE = 0.1;
    public static double MAX_POWER_SLIDE = 1;

    //Slide heights
    public static int MIN_HEIGHT = 0;
    public static int LOW_HEIGHT = 515;
    public static int MEDIUM_HEIGHT = 1600;
    public static int MAX_HEIGHT = 3400;
    public static int INCREMENT_STEPS_SLIDE = 40;
    public static double INCHESPERTICK = 16.75/1835;

    //Rot powers
    public static double MIN_POWER_ROT = 0;
    public static double HOLD_POWER_ROT = 1;
    public static double MAX_POWER_ROT = 1;

    //Rot steps
    //public static int MAX_ROT = 5000;
    //public static int HIGH_ROT = 5000;
    //public static int MEDIUM_ROT = 1295;
    //public static int LOW_ROT = 1060;
    //public static int MIN_ROT = 1060;

    public static int MIN_ROT = 0;
    public static int LOW_ROT = 1060;
    public static int MEDIUM_ROT = 1295;
    public static int HIGH_ROT = 840;
    public static int MAX_ROT = 2106;
    
    //Rot constants
    public static int INCREMENT_ROT = 10;
    public static int TICKSPERROT = 2520;
    public static double DEGPERTICK = 90.0/500.0;
    
    //Grip pos
    public static double RFLOOR = 0;
    public static double RPLACE = 0;
    public static double RWALL = 1;

    public static double RCLOSE = 0;
    public static double ROPEN = 1;
    public static double LCLOSE = 0;
    public static double LOPEN = 1;
    //motors
    public DcMotorEx slideMotor;
    public DcMotorEx rotMotor;

    //Servos that are a part of the robot
    public Servo leftGripper, rightGripper, rotServo, droneServo;

    public DigitalChannel leftLED;
    public DigitalChannel rightLED;


    private int armMotorSteps = 0;
    private int rotMotorSteps = 0;

    public boolean place = false;

    public LinearSlide(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slidemotor");

        slideMotor.setDirection(DcMotorEx.Direction.FORWARD);
        slideMotor.setTargetPosition(MIN_HEIGHT);
        slideMotor.setPower(MAX_POWER_SLIDE);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rotMotor = hardwareMap.get(DcMotorEx.class, "rotmotor");
        rotMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rotMotor.setTargetPosition(MIN_ROT);
        rotMotor.setPower(MAX_POWER_ROT);
        rotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftGripper = hardwareMap.get(Servo.class, "gripL");
        rightGripper = hardwareMap.get(Servo.class, "gripR");

        droneServo = hardwareMap.get(Servo.class, "droneLauncher");

        rotServo = hardwareMap.get(Servo.class, "rotR");

        leftLED = hardwareMap.get(DigitalChannel.class, "left");
        rightLED = hardwareMap.get(DigitalChannel.class, "right");
        leftLED.setMode(DigitalChannel.Mode.OUTPUT);
        rightLED.setMode(DigitalChannel.Mode.OUTPUT);


    }

    public void setSlide(int steps) {
        armMotorSteps = Range.clip(steps, MIN_HEIGHT, MAX_HEIGHT);
        slideMotor.setTargetPosition(armMotorSteps);
    }

    public void setRot(int steps) {
        rotMotorSteps = Range.clip(steps, MIN_ROT - 10000, MAX_ROT + 100);
        rotMotor.setTargetPosition(rotMotorSteps);
    }

    public void setArmPos(int armstep, int rotstep) {
        setRot(rotstep);
        setSlide (armstep);
    }

    public void setAutoPos (int armstep, int rotstep) {
        setArmPos(armMotorSteps,rotstep);
        ElapsedTime safetime = new ElapsedTime();
        safetime.reset();
        while (rotMotor.isBusy() && safetime.time() < 3) {
                rotMotor.setPower(MAX_POWER_ROT);
        }
        rotMotor.setPower(HOLD_POWER_ROT);
        if (rotMotor.getCurrentPosition() == MIN_ROT) {
            rotMotor.setPower(MIN_POWER_ROT);
        }
        setArmPos(armstep,rotstep);
        while (slideMotor.isBusy()) {
            slideMotor.setPower(MAX_POWER_SLIDE);
        }
        slideMotor.setPower(HOLD_POWER_SLIDE);
        if (slideMotor.getCurrentPosition() == MIN_HEIGHT) {
            slideMotor.setPower(MIN_POWER_SLIDE);
        }
    }

    public void setAutoExtendo (int armstep) {
        int rotstep =  Range.clip((int)((( Math.toDegrees(Math.acos(8.5/(9 + 4 + (4 * Range.clip(armstep - 560, 0, 1000000000)/340.0)))))-50)/(LinearSlide.DEGPERTICK) ), 0, 500);
                //Range.clip((int)((( Math.toDegrees(Math.acos(8.5/(14 + (INCHESPERTICK * Range.clip(armMotorSteps - 240, 0, 1000000000)))))) - 45)/(LinearSlide.DEGPERTICK) ), 0, 500);


        if (armstep > armMotorSteps) {
            setArmPos(armMotorSteps,rotstep);
            ElapsedTime safetime = new ElapsedTime();
            safetime.reset();
            while (rotMotor.isBusy() && safetime.time() < 3) {
                rotMotor.setPower(MAX_POWER_ROT);
            }
            rotMotor.setPower(HOLD_POWER_ROT);
            if (rotMotor.getCurrentPosition() == MIN_ROT) {
                rotMotor.setPower(MIN_POWER_ROT);
            }
            setArmPos(armstep,rotstep);
            while (slideMotor.isBusy()) {
                slideMotor.setPower(MAX_POWER_SLIDE);
            }
            slideMotor.setPower(HOLD_POWER_SLIDE);
            if (slideMotor.getCurrentPosition() == MIN_HEIGHT) {
                slideMotor.setPower(MIN_POWER_SLIDE);
            }
        } else {
            setArmPos(armstep,rotMotorSteps);
            ElapsedTime safetime = new ElapsedTime();
            safetime.reset();
            while (slideMotor.isBusy() && safetime.time() < 3) {
                slideMotor.setPower(MAX_POWER_SLIDE);
            }
            slideMotor.setPower(HOLD_POWER_SLIDE);
            if (slideMotor.getCurrentPosition() == MIN_ROT) {
                slideMotor.setPower(MIN_POWER_SLIDE);
            }
            setArmPos(armstep,rotstep);
            while (rotMotor.isBusy()) {
                rotMotor.setPower(MAX_POWER_ROT);
            }
            rotMotor.setPower(HOLD_POWER_ROT);
            if (rotMotor.getCurrentPosition() == MIN_ROT) {
                rotMotor.setPower(MIN_POWER_ROT);
            }
        }


    }

    public void update() {
        // If motor is busy, run motor at max power.
        // If motor is not busy, hold at current position or stop at lowest height

        if (slideMotor.isBusy()) {
            slideMotor.setPower(MAX_POWER_SLIDE);
        } else {
            slideMotor.setPower(HOLD_POWER_SLIDE);
            if (slideMotor.getCurrentPosition() == MIN_HEIGHT) {
                slideMotor.setPower(MIN_POWER_SLIDE);
            }
        }

//        if (!place) {
//            turnFloorEx();
//        } else {
//            turnPlaceEx();
//        }
    }
//Grab functions
    public void ungrabL() {
        leftGripper.setPosition(LCLOSE);
        leftLED.setState(false);
    }
    public void grabL() {
        leftGripper.setPosition(LOPEN);
        leftLED.setState(true);
    }
    public void ungrabR() {
        rightGripper.setPosition(ROPEN);
        rightLED.setState(false);
    }
    public void grabR() {
        rightGripper.setPosition(RCLOSE);
        rightLED.setState(true);
    }

    public void ungrabAll() {
        ungrabL();
        ungrabR();
        rightLED.setState(false);
        leftLED.setState(false);
    }
    public void grabAll() {
        grabL();
        grabR();
        rightLED.setState(true);
        leftLED.setState(true);
    }



    public void turnRot(Servo rotX, double ticks) {
        rotX.setPosition(Range.clip(ticks, 0, 1));
    }

    public void turnFloor() {
        turnRot(rotServo, RWALL);
    }

    public void turnPlace() {
        turnRot(rotServo, RPLACE);
    }

    public void turnPlaceEx() {
        if (rotMotor.getCurrentPosition() > HIGH_ROT) {
            turnRot(rotServo, RPLACE + (DEGPERTICK * (1.0/270.0) * (rotMotor.getCurrentPosition() - (1090))));
        } else {
            turnPlace();
        }
    }

    public void turnFloorEx() {
        if (rotMotor.getCurrentPosition() < HIGH_ROT - 100) {
            turnRot(rotServo, RFLOOR + (DEGPERTICK * (1.0/270.0) * (rotMotor.getCurrentPosition())));
            } else {
            turnFloor();
        }
    }
    public void turnPlaceAuto() {
        turnRot(rotServo, RPLACE);
    }

    public void turnServoRot(double position){
        rotServo.setPosition(position);
    }

    public static double slideDeadband(double input, double deadbandThreshold) {
        // If the absolute value of input is within the deadband, return 0
        if (Math.abs(input) <= deadbandThreshold) {
            return 0.0;
        }
        // Determine the sign and magnitude outside the deadband
        return input;
    }

    public void zero(){
        rotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(slideDeadband(rotMotor.getVelocity(), 0.3)!=0){
            rotMotor.setTargetPosition(-20);
            rotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
