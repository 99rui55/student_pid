
package frc.robot;

public class Trapez {
    double maxVelocity;
    double maxAcceleration;
    private double deltaVelocity;

    public Trapez(double maxVelocity, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        deltaVelocity = maxAcceleration * 0.02;

    }

    private double distanceToVelocity(double currentVelocity, double targetVelocity, double acceleration) {
        double deltaVelocity = currentVelocity - targetVelocity;
        return (currentVelocity - deltaVelocity/2)*deltaVelocity/acceleration;
    }

    public double calculate(double remainingDistance, double curentVelocity, double targetVelocity) {
        if(remainingDistance < 0) {
            return  -1*calculate(-1*remainingDistance, -1*curentVelocity, -1*targetVelocity);
        }
        if(curentVelocity < maxVelocity && distanceToVelocity(curentVelocity+deltaVelocity, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceWithAccel(curentVelocity)) {
            return Math.min(curentVelocity + deltaVelocity, maxVelocity);

        } else if(distanceToVelocity(curentVelocity, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceNoAccel(curentVelocity)) {
            return curentVelocity;
        } else {
            return Math.max(curentVelocity - deltaVelocity,targetVelocity);
        }
        
    }
    
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * 0.02;
    }
    private double cycleDistanceWithAccel(double currentVelocity) {
        return currentVelocity * 0.02 + (maxAcceleration * 0.0002);
    }

    

    
    
}