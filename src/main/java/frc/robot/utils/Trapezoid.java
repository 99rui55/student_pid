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
        if(remainingDistance >= neededDistToGetToV(nextCycleV(currentV), endV)){
            System.out.println("accelerating or preserving speed");
            System.out.println("next speed: " + Math.min(currentV + maxA*cycleTime, maxV));
            return Math.min(currentV + maxA*cycleTime, maxV);
        }
        // else if(remainingDistance <= neededDistToGetToV(currentV, endV)){
        //     System.out.println("preserving speed");
        //     return currentV;
        // }
        else{
            System.out.println("slowing down");
            System.out.println("current V: " + currentV + ", maxA*cycletime: "  + maxA*cycleTime);
            System.out.println("next speed: " + (currentV - maxA*cycleTime));
            return (currentV - maxA*cycleTime);
        }
    }
    public double nextCycleV(double currentV){
        return currentV + maxA * cycleTime;
    }
    
    public double neededDistToGetToV(double currentV, double endV){
        return ((currentV*currentV - endV*endV) / 2 * maxA);
    }
}
