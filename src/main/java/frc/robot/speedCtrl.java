package frc.robot;


import java.lang.management.OperatingSystemMXBean;

import frc.robot.Constants.OperatorConstants;

public class speedCtrl {

    double maxVel;
    double maxAcc;

    /**
     * 
     * @param maxVel - Maximum Velocity (Meters per second)
     * @param maxAcc - Maximum Acceleration (Meters per second)
     */
    public speedCtrl(double maxVel, double maxAcc)
    {
        this.maxAcc = maxAcc;
        this.maxVel = maxVel;
    }

    /**
     * 
     * @param rD - Remaining Distance (Meters)
     * @param cVel - Current Velocity (Meters per Second)
     * @param tgtVel - Target Velocity (Meters per second)
     * @param endVel - End Velocity (Meters per second)
     * @return Returns the velocity (Meters per cycle time) needed in the current cycle in order to mantain a Trapzoid Motion Profile.
     * According to the remaining distance, given the current velocity, target velocity, and end velocity.
     */
    public double calculate(double rD /**Remaining Distance*/,double cVel, double tgtVel, double endVel)
    {
        //Acceleration and the next velocity are in Meters per cycle time, not seconds.
        double acc = Math.signum(tgtVel - cVel) * this.maxAcc * OperatorConstants.cTime;
        double nextV = cVel * OperatorConstants.cTime + acc;

        if(rD - accDC(nextV / OperatorConstants.cTime, tgtVel) > accD(nextV / OperatorConstants.cTime, tgtVel))
        {
            if(Math.abs(tgtVel - nextV / OperatorConstants.cTime) > this.maxAcc){
                System.out.println("Acc to Target Speed");
                return nextV;
            }
            else{
                System.out.println("Keep Target Speed");
                return tgtVel * OperatorConstants.cTime;
            }
        }
        else{
            acc = Math.signum(endVel - cVel) * this.maxAcc * OperatorConstants.cTime;
            nextV = cVel * OperatorConstants.cTime + acc;
            if(Math.abs(endVel - nextV / OperatorConstants.cTime) > this.maxAcc)
            {
                System.out.println("Acc to End Speed");
                return nextV;
            }
            else{
                System.out.println("Keep End Speed");
                return endVel * OperatorConstants.cTime;
            }
        }
    }

   /**
    * The distance required to accelerate from current velocity to a target velocity,
    * given the maximum acceleration.
    * @param cVel - Current Velocity (Meters per seccond)
    * @param tgtVel - Target Velocity (Meters per second)
    * @return The distance required to accelerate from current velocity to a target velocity,
    * given the maximum acceleration.
    */
    double accD(double cVel,double tgtVel)
    {
        double acc = Math.signum(tgtVel - cVel) * maxAcc;
        double time = (tgtVel - cVel)/acc;
        
        return time * cVel + 0.5 * acc * Math.pow(time, 2);
    }

    /**
     * 
     * @param cVel Current Velocity (Meters per second)
     * @param tgtVel Target Velocity (Meters per second)
     * @return The distance passed after one cycle, given the current velocity and the maximum acceleration.
     */
    double accDC(double cVel,double tgtVel)
    {
        double acc = Math.signum(tgtVel - cVel) * maxAcc;
        return OperatorConstants.cTime * cVel + 0.5 * acc * Math.pow(OperatorConstants.cTime, 2);
    }

}
