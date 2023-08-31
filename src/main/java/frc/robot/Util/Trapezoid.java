// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Constants;

/** Add your docs here. */
public class Trapezoid {
    private double maxV;
    private double maxA;
    private double maxDis;
    public Trapezoid(double maxV, double maxA, double maxDis){
        this.maxA = maxA;
        this.maxV = maxV;
        this.maxDis = maxDis;
    }

    public double calculate(double remainingDis, double currentV, double endV){
        if((currentV<this.maxV)&&(remainingDis>neededDis(remainingDis)))
            return Math.min(currentV+this.maxA*Constants.cicleTime , this.maxV);
    }
    public double neededDis(){
        double d1 = 
        double d2 = this.maxDis - d1*2;
        return this.maxDis
    }
}
