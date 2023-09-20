package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.Trapez;

public class udi extends CommandBase{
  private Chassis chassis;
    Trapez Trapez;
    private double distance;
    private double direction;
    private double startDistance;
    private double remainingDistance;


    public udi(double distance, double velocity, double maxAcceleration, Chassis chassis) {
        super();
        this.chassis = chassis;
        this.distance = distance;
        direction  = Math.signum(distance);
        Trapez = new Trapez(velocity, maxAcceleration);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        startDistance = chassis.getDistance();
    }

    @Override
    public void execute() {
        remainingDistance = remainingDistance();
        double v = Trapez.calculate(remainingDistance, chassis.getVelocity(), 0.2);
        chassis.setVelocity(v, v);
    }

    private double remainingDistance() {
        return direction*(distance + startDistance - chassis.getDistance());
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setVelocity(0, 0);
    }

    @Override
    public boolean isFinished() {
        return remainingDistance < 0.02;
    }

}