package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Trapezoid {
    private final double maxVelocity;
    private final double acceleration;
    private final double distance;
    private final Translation2d startPos;
    
    private double desiredVelocity;

    public Trapezoid(double maxVelocity, double acceleration, double distance, Translation2d startPos) {
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.distance = distance;
        this.startPos = startPos;

        desiredVelocity = 0;
    }

    public void update(double currentVelocity, Translation2d currentPosition) {
        double distancePassed = startPos.getDistance(currentPosition);
        double distanceLeft = distance - distancePassed;
        double accelDistance = acceleration*2 / Math.pow(maxVelocity, 2);
        if (distanceLeft <= accelDistance) {
            // deccelerate
            desiredVelocity -= acceleration;
        }
        else if (currentVelocity < maxVelocity) {
            // accelerate
            desiredVelocity += acceleration;
        }
        else {
            // keep velocity
            desiredVelocity = maxVelocity;
        }
    }
    
    public double getDesiredVelocity() {
        return desiredVelocity;
    }

    public boolean isFinished(Translation2d currentPosition) {
        return startPos.getDistance(currentPosition) >= distance;
    } 
}
