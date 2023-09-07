package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.Trapez;

public class udi extends CommandBase{
  private Chassis chassis;
  private double distance;
  private double Acceleration;
  private double velocity;
  private Trapez Trapez;
  private double distancleft;
  public udi(Chassis chassis, double distance, double velocity, double Acceleration){
    this.chassis = chassis;
    this.distance = distance;
    this.velocity = velocity;
    this.Acceleration = Acceleration;
    Trapez = new Trapez(velocity, Acceleration);
    addRequirements(chassis);

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = chassis.getDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distancleft = Math.abs(chassis.getDistance() - distance);
    velocity = Trapez.calculate(distancleft, chassis.getVelocity(), 0 );
    chassis.setVelocity(velocity, velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setVelocity(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}