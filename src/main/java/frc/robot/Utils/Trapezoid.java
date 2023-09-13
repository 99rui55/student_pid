// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import frc.robot.Constants;

/** Add your docs here. */
public class Trapezoid {
    public double maxVel;
    public double maxAcc;
    public double neededDis;
    public double time;
    public String state;

    public Trapezoid(double maxVel, double maxAcc){
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        time = maxVel / maxAcc;
        neededDis = 0.5* maxAcc*time*time;
        state = "prepare for trapezoid / error";
    }

    public double calculate(double remainingDis, double currentVel, double endVel) {

        if ((currentVel < maxVel) && (remainingDis > neededDis)) {
            // when the velocity of the chassis is less than the max and there is place to increase the speed
            // return the increased velocity or the max velocity if we closer to that
            state = "phase 1 - accelaration";
            return Math.min(currentVel + maxAcc * Constants.cycleTime, maxVel);

        } else if (remainingDis >= neededDis) {
            // when the remining distence is bigger or worth the needed distence and the velocity is the max velocity
            // return the max velocity to continue moving at the same speed
            state = "phase 2 - stay at max veloicty";
            return maxVel;

        } else if (currentVel > endVel) {
            // when the distence is already at the third phase at the decreasing velocity phase
            // return the decreased velocity or the end velocity if we closer to that
            state = "phase 3 - deceleration";
            double help = currentVel;
            if (currentVel > maxVel){
                help = maxVel;
            }
            return Math.max(help - maxAcc * Constants.cycleTime, endVel);
            

        } else{
            // when we at the end of the trapezoid we only use the end velocity so we
            // return the end velocity
            state = "out of trapezoid / error";
            return endVel;
        }
    }
}
