package frc.robot.utils;
import static frc.robot.Constants.GeneralConstants.*;

public class Trapezoid {
    private double maxA;
    private double maxV;
    public Trapezoid(double maxA, double maxV) {
        this.maxA  = maxA;
        this.maxV = maxV;
    }

    public double calculate(double remainingDistance, double currentV, double endV){
        if(remainingDistance > neededDistToGetToV(nextCycleV(currentV), endV))
            return Math.min(currentV + maxA*cycleTime, maxV);
        else if(remainingDistance <= neededDistToGetToV(currentV, endV))
            return currentV;
        else
            return currentV - maxA*cycleTime;
    }
    
    public double nextCycleV(double currentV){
        return currentV + maxA * cycleTime;
    }
    
    public double neededDistToGetToV(double currentV, double endV){
        return ((currentV*currentV - endV*endV) / 2 * maxA);
    }
}
