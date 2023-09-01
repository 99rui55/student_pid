package frc.robot;

import static frc.robot.Constants.OperatorConstants;

import frc.robot.Constants.OperatorConstants;

class speedCtrl {

    double maxVel;
    double maxAcc;

    speedCtrl(double maxVel, double maxAcc)
    {
        this.maxAcc = maxAcc;
        this.maxVel = maxVel;
    }

    double calculate(double rD /**Remaining Distance*/,double cVel, double tgtVel, double endVel)
    {

        //Acceleration according to cycle time and direction (backward, forward)
        double acc = Math.signum(cVel) * maxAcc * OperatorConstants.cTime;


        //Make sure that the target vel does not exceed the max vel
        if(tgtVel > this.maxVel)
            tgtVel = this.maxVel;

        //Next vel in next cycle = current vel plus acc
        double nextV = cVel + acc;

        /*Check if the remaining distance in the next cycle (with the current vel plus acc)
         will be enough for the robot to deaccelerate to the requested vel at the end   */
        if(rD -  accDC(cVel) > accD(nextV, endVel))
            //Check if current vel doesn't exceed the target vel
            if(Math.abs(cVel) < tgtVel)
                //If the vel is less than the target vel, return the vel added with acc
                return nextV;
            else
                return tgtVel;
        else
        {
            //Since there will be not enough distance to deaccelerate to the end vel in the next cycle
            //(if we decided to keep the current vel).
            //Then, the robot should deaccelerate beforehand.
            nextV = cVel - acc;
            return nextV;
        }
    }

    double accD(double cVel,double tgtVel)
    {
        double time = (tgtVel - cVel)/maxAcc;
        double acc;
        //Checking the direction of the acceleration (backward,forward)
        if(cVel > tgtVel)
            acc = -maxAcc;
        else
            acc = maxAcc;
        
        return time * cVel + 0.5 * acc * Math.pow(time, 2);
    }

    double accDC(double cVel)
    {
        double acc = Math.signum(cVel) * maxAcc;
        return OperatorConstants.cTime * cVel + 0.5 * acc * Math.pow(OperatorConstants.cTime, 2);
    }

}
