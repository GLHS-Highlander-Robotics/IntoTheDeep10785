package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double target_pos=0;
    public double integral_sum=0;
    public double derivative=0;

    double dErr=0, dt=0, lastErr=0, lastT=0;

    double Kp=0.1;
    double Ki=0;
    double Kd=0;


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

    public PID(double t_pos){
       target_pos= t_pos;
    }

    public double error(double system){
        return system-target_pos;
    }

    public double delta_T(double time){
        double dT=time-lastT;
        lastT=time;
        return dT;
    }

    public double calculate(double system, double time){
        dt=delta_T(time);
        integral_sum+=error(system)*dt;
        derivative=(error(system-lastErr))/dt;
        lastErr=error(system);
        return Kp*error(system) + Ki*integral_sum + Kd*derivative;
    }
    public void reset(){
        target_pos=0;
        integral_sum=0;
        derivative=0;
        dErr=0;
        dt=0;
        lastErr=0;
        lastT=0;
    }
    
}
