package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveAuto extends CommandBase {
  private final Chassis chassis;

  public DriveAuto(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.setVelocity(0.4, 0.4);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
