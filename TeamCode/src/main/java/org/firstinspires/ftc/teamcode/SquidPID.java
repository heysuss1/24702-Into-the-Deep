package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;

public class SquidPID extends PIDFController {
    public SquidPID(double kP, double kI, double kD, double kF){
        super(kP, kI, kD, kF);
    }
    public void setPIDF(double kP, double kI, double kD, double kF){
        setP(kP);
        setI(kI);
        setD(kD);
        setF(kF);
    }
    public double calculate(double pv, double error){
        double calc = super.calculate(pv, error);
        double sign = 1;
        if (calc < 0){
            sign = -1;
        }
        return Math.sqrt(Math.abs(calc)) * sign;
    }
    public void reset(){
        super.reset();
    }
}
