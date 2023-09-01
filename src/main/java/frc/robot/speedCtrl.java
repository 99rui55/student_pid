package frc.robot;


import frc.robot.Constants.OperatorConstants;

public class speedCtrl {

    double maxVel;
    double maxAcc;

    public speedCtrl(double maxVel, double maxAcc)
    {
        this.maxAcc = maxAcc;
        this.maxVel = maxVel;
    }

    /**
     * 
     * @param rD - Remaining Distance
     * @param cVel - Current Velocity
     * @param tgtVel - Target Velocity
     * @param endVel - End Velocity
     * @return Returns the velocity needed in the current cycle in order to mantain a Trapzoid Motion Profile.
     * According to the remaining distance, given the current velocity, target velocity, and end velocity.
     */
    public double calculate(double rD /**Remaining Distance*/,double cVel, double tgtVel, double endVel)
    {

        //Acceleration according to cycle time and direction (backward, forward)
        double acc = Math.signum(tgtVel - cVel) * maxAcc * OperatorConstants.cTime;


        //Make sure that the target vel does not exceed the max vel
        if(tgtVel > this.maxVel)
            tgtVel = this.maxVel;

        //Next vel in next cycle = current vel plus acc
        double nextV = cVel + acc;

        /*Check if the remaining distance in the next cycle (with the current vel plus acc)
         will be enough for the robot to deaccelerate or accelerate to the requested vel at the end   */
        if(rD -  accDC(cVel,endVel) > accD(nextV, endVel))
            //Check if the velocity in the next cycle doesn't exceed the target vel
            if(Math.abs(nextV) < Math.abs(tgtVel))
                //If the vel is less than the target vel, return the vel added with acc
                return nextV;
            else
                return tgtVel;
        else
        {
            //Since there will be not enough distance to deaccelerate or accelerate to the end vel in the next cycle
            //(if we decided to keep the current vel).
            //Then, the robot should deaccelerate or accelerate beforehand.
            acc = Math.signum(endVel - cVel) * maxAcc * OperatorConstants.cTime;
            nextV = cVel + acc;
            if(Math.abs(nextV) < Math.abs(endVel))
                return nextV;
            else
                return endVel;
        }
    }

   /**
    * The distance required to accelerate from current velocity to a target velocity,
    * given the maximum acceleration.
    * @param cVel - Current Velocity
    * @param tgtVel - Target Velocity
    * @return The distance required to accelerate from current velocity to a target velocity,
    * given the maximum acceleration.
    */
    double accD(double cVel,double tgtVel)
    {
        double acc;
        //Checking the direction of the acceleration (backward,forward)
        if(cVel > tgtVel)
            acc = -maxAcc;
        else
            acc = maxAcc;
        double time = (tgtVel - cVel)/acc;
        
        return time * cVel + 0.5 * acc * Math.pow(time, 2);
    }

    /**
     * 
     * @param cVel Current Velocity
     * @return The distance passed after one cycle, given the current velocity and the maximum acceleration.
     */
    double accDC(double cVel,double tgtVel)
    {
        double acc = Math.signum(tgtVel - cVel) * maxAcc;
        return OperatorConstants.cTime * cVel + 0.5 * acc * Math.pow(OperatorConstants.cTime, 2);
    }

}
