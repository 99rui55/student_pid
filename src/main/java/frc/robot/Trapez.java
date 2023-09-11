
package frc.robot;

public class Trapez {
    double maxVelocity;
    double maxAcceleration;
    private double accelDist;
    private double deltaV;

    public Trapez(double maxVelocity, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        deltaV = maxAcceleration * 0.02;
        accelDist = maxAcceleration * 0.0002;

    }

    private double cycleDistanceWithAccel(double currentVelocity) {
        return currentVelocity * 0.02 + accelDist;
    }
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * 0.02;
    }

    private double distanceToVel(double currentVel, double tgtVel, double accel) {
        double deltaVel = currentVel - tgtVel;
        return (currentVel - deltaVel/2)*deltaVel/accel;
    }

    public double calculate(double remainingDistance, double curentVelocity, double tgtVelocity) {
        if(remainingDistance < 0) {
            return  -1*calculate(-1*remainingDistance, -1*curentVelocity, -1*tgtVelocity);
        }
        if(curentVelocity < maxVelocity && distanceToVel(curentVelocity+deltaV, tgtVelocity, maxAcceleration) < remainingDistance - cycleDistanceWithAccel(curentVelocity)) {
            return Math.min(curentVelocity + deltaV, maxVelocity);

        } else if(distanceToVel(curentVelocity, tgtVelocity, maxAcceleration) < remainingDistance - cycleDistanceNoAccel(curentVelocity)) {
            return curentVelocity;
        } else {
            return Math.max(curentVelocity - deltaV,tgtVelocity);
        }
        
    }

    
    
}