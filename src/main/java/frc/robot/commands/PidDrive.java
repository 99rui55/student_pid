package frc.robot.commands;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class PidDrive extends CommandBase {
  Chassis chassis;
  double wantedVelocity;

  public double getWantedVelocity(){
    return wantedVelocity;
  }
  public void setWantedVelocity(double wantedVelocity){
    this.wantedVelocity = wantedVelocity;
  }

  public PidDrive() {
    addRequirements(chassis);
  }
  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("Current Velocity", chassis::getVelocity, null);
    builder.addDoubleProperty("Wanted Velocity", this::getWantedVelocity, this::setWantedVelocity);
  }

  @Override
  public void initialize() {
    
  }



  @Override
  public void execute() {
    chassis.setVelocity(wantedVelocity);
  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
