package org.firstinspires.ftc.teamcode;

public class PIDController {

    double integral, derivative;
    double target, kp, ki, kd;
    double proportionOld = 0;
    double oldTime = System.currentTimeMillis();

    static double currentTime = System.currentTimeMillis();

    public PIDController(double target, double kp, double ki, double kd) {
        this.target = target;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

    }

    public double calculateResultant(double presentValue, double targe) {
        currentTime = System.currentTimeMillis();

        double deltaT = currentTime-oldTime;

        this.target = target;
        double proportion = target - presentValue;
        integral += proportion * deltaT;
        derivative = (proportion-proportionOld)/deltaT;
        proportionOld = proportion;


        oldTime = currentTime;

        return kp * proportion + ki * integral + kd * derivative;
    }


}
