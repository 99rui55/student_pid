// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Constants;

/** Add your docs here. */
public class Trapezoid {
    private double maxV;
    private double maxA;

    public Trapezoid(double maxV, double maxA) {
        this.maxA = maxA;
        this.maxV = maxV;
    }

    public double calculate(double remainingDis, double currentV, double endV) {
        if ((currentV < this.maxV) && (remainingDis > neededDis())) {
            System.out.println("true1--------");
            return Math.min(currentV + this.maxA * Constants.cycleTime, this.maxV);
        } else if (remainingDis >= neededDis()) {
            System.out.println("true2--------");
            return this.maxV;
        } else if (currentV > endV) {
            System.out.println("true3--------");
            System.out.println(this.maxA * Constants.cycleTime);
            System.out.println(currentV - this.maxA * Constants.cycleTime);
            //return 0;
            return Math.max(currentV - this.maxA * Constants.cycleTime, endV);
        }
        System.out.println("true4--------");
        return endV;
    }

    public double neededDis() {
        double t = this.maxV / this.maxA;
        double d1 = 0.5 * this.maxA * t * t;
        return d1;
    }
}
