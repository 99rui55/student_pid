// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Constants;

/** Add your docs here. */
public class Trapezoid {
    private double maxV;
    private double maxA;
    public Trapezoid(double maxV, double maxA){
        this.maxA = maxA;
        this.maxV = maxV;
    }

    public double calculate(double remainingDis, double currentV, double endV){
        if((currentV<this.maxV)&&(remainingDis>neededDis())){
            return Math.min(currentV+this.maxA*Constants.cicleTime , this.maxV);
        } else if(remainingDis>neededDis()){
            return currentV;
        }else if(currentV>endV){
            return Math.min(currentV-this.maxA*Constants.cicleTime , endV);
        }
        return endV;
    }
    public double neededDis(){
        double t = this.maxV/this.maxA;
        double d1 = 0.5*this.maxA*t*t;
        return d1;
    }
}
