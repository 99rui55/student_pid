package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.VelocityPID;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
  private final Chassis chassis;

  public RobotContainer() {
    chassis = new Chassis();
    SmartDashboard.putData("Drive PID Command", new VelocityPID(chassis).withTimeout(3));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
