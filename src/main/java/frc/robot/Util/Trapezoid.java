package frc.robot.Util;

import static frc.robot.Constants.ChassisConstants.*;

public class Trapezoid {

    private double maxV;
    private double maxA;

    public Trapezoid(double maxV, double maxA){
        this.maxA = maxA;
        this.maxV = maxV;
    }

    public double calculate(double remainingDistance, double currentV, double endV) {
        if (velocity_after_accel(currentV) <= maxV && distToVel(velocity_after_accel(currentV), endV, maxA) <= remainingDistance) {  // Can I accelerate?
            return velocity_after_accel(currentV); // if not at max speed AND can stop on time, then accelerate
        }
        else if (distToVel(velocity_after_accel(currentV), endV, maxA) <= remainingDistance) { 
            return maxV; // if can stop on time AND at max speed, then keep maxV
        }
        else { // else (if cant stop on time, assuming i accelerate), then deccelerate
            return velocity_after_deccel(currentV);
        }
    }

    public double velocity_after_accel(double currentV) {
        return (currentV + maxA * circleTime);
    }

    public double velocity_after_deccel(double currentV) {
        return (currentV + (-maxA) * circleTime);
    }

    public double distToVel(double currentV, double endV, double maxA) {    // TAB 1
        return (Math.pow(maxV, 2)-Math.pow(currentV, 2) / (2*maxA));
    }
}
