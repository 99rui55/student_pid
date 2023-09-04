package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class VelocityPID extends CommandBase {
  private final Chassis chassis;

  private double velocity;

  public VelocityPID(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    SmartDashboard.putData(this);
  }

  private double getVelocity() {
    return velocity;
  }

  private void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Velocity", chassis::getVelocity, null);
    builder.addDoubleProperty("Set Point",this::getVelocity, this::setVelocity);
  }

  @Override
  public void initialize() {
    chassis.modifyMotorConfig(velocity / 2);
    chassis.setVelocity(velocity, velocity);
  }

  @Override
  public void end(boolean interrupted) {
      chassis.setVelocity(velocity, velocity);
  }
}
