package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.Trapez;

public class udi extends CommandBase{
  private Chassis chassis;
    Trapez Trapez;
    private double distance;
    private double direction;
    private double heading = 0;
    private double startDistance = 0;
    private double remainingDistance = 0;

    private final double headingKP = 0.1;

    public udi(double distance, double velocity, double maxAcceleration, Chassis chassis) {
        super();
        this.chassis = chassis;
        this.distance = distance;
        direction = Math.signum(distance);
        Trapez = new Trapez(velocity, maxAcceleration);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        heading = chassis.getrotation();
        startDistance = chassis.getDistance();
    }

    @Override
    public void execute() {
        double curentVelocity = chassis.getVelocity();
        double headingError = chassis.getrotation() - heading;
        remainingDistance = remainingDistance();
        double tgtVel = Trapez.calculate(remainingDistance, curentVelocity, 0) * direction;
        chassis.setVelocity(tgtVel + headingError * headingKP, tgtVel - headingError * headingKP);
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