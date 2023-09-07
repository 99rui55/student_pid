
package frc.robot;


/** Add your docs here. */
public class Trapez {
    private double maxAcceleration;
    private double maxVelocity;
    private double distance;
    public Trapez(double maxVelocity, double maxAcceleration){
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
    }

    public double neededdistancets(){
        distance = (Math.pow(maxVelocity,2)) / (2 * maxAcceleration);
        return distance;
    }

    public double calculate(double rdistance, double Velocity, double endVelocity){
        if(rdistance>neededdistancets()){
            return Math.min(Velocity+maxAcceleration*0.02 , maxVelocity);
        }
        else{
            return Math.max(Velocity-maxAcceleration*0.02 , endVelocity);
        }
    }
    
}