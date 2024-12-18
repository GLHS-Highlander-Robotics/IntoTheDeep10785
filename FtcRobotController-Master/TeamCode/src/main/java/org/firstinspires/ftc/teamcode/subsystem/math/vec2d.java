package org.firstinspires.ftc.teamcode.subsystem.math;

public class vec2d {
    public double mag, dir, rot;
    public double x, y, theta;

    //Just add an extra "false" to make the vector be initialized with polar form
    public vec2d(double mag_i,double dir_i,double rot_i,boolean magdir_form){
        mag = mag_i;
        dir = dir_i;
        rot = rot_i;

        x = mag_i*Math.sin(Math.toRadians(dir_i));
        y = mag_i*Math.cos(Math.toRadians(dir_i));
        theta = rot_i;
    }

    public vec2d(double x_i,double y_i,double theta_i){
        x=x_i;
        y=y_i;
        theta=theta_i;

        mag=Math.sqrt(x_i*x_i+y_i*y_i);
        dir=Math.toDegrees(Math.atan2(x_i, y_i));
        rot = theta_i;
    }

    public boolean approx_equal(double x_i,double y_i,double theta_i){
        return Math.abs(x_i - x) <= 1 && Math.abs(y_i - y) <= 1 && Math.abs(theta_i - theta) <= 4;
        //tolerances: 1 in both ways, 4 degrees heading
    }

    public void normalize(){
        mag = 1;

        x = Math.sin(Math.toRadians(dir));
        y = Math.cos(Math.toRadians(dir));
        theta = rot;
    }

}
