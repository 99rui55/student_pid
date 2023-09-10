package frc.robot.Util;

import static frc.robot.Constants.ChassisConstants.*;

import frc.robot.subsystems.Chassis;

public class Trapezoid {

    private double maxV, maxA, remainingDistance, endV, distanceTraveled = 0;
    private boolean first = true;

    public Trapezoid(double maxV, double maxA, double remainingDistance, double endV){
        this.maxV = maxV;
        this.maxA = maxA;
        this.remainingDistance = remainingDistance;
        this.endV = endV;

    }

    public double calculate(double remainingDistance, double currentV, double endV) {
        if (first) return accelerate(currentV);
        if (accelerate(currentV) <= maxV && distToVel(accelerate(currentV), endV, maxA) <= remainingDistance) {  // Can I accelerate?
            return accelerate(currentV); // if not at max speed AND can stop on time, then accelerate
        }
        else if (distToVel(accelerate(currentV), endV, maxA) <= remainingDistance) { 
            return maxV; // if can stop on time AND at max speed, then keep maxV
        }
        else { // else (if cant stop on time, assuming i accelerate), then deccelerate
            return deccelerate(currentV);
        }
    }

    public double accelerate(double currentV) {
        return (currentV + maxA * circleTime);
    }
    
    public double deccelerate(double currentV) {
        return (currentV + (-maxA) * circleTime);
    }

    public double distToVel(double currentV, double endV, double maxA) {    // TAB 1
        return (Math.pow(maxV, 2)-Math.pow(currentV, 2) / (2*maxA));
    }
}
