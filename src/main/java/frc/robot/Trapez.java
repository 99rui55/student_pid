
package frc.robot;


/** Add your docs here. */
public class Trapez {
    private double maxAcceleration;
    private double maxVelocity;
    private double time;
    private double distance;
    public Trapez(double maxVelocity, double maxAcceleration){
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
    }

    public double ndistance(){
        time = maxVelocity/maxAcceleration;
        distance = (maxAcceleration*Math.pow(time,2)/2);
        return distance;
    }

    public double calculate(double rdistance, double Velocity, double endVelocity){
        if((Velocity<maxVelocity)&&(rdistance>ndistance())){
            return Math.min(Velocity+maxAcceleration*0.02 , maxVelocity);
        } 
        else if(rdistance>ndistance()){
            return Velocity;
        }
        else{
            return Math.max(Velocity-maxAcceleration*0.02 , endVelocity);
        }
    }
    
}