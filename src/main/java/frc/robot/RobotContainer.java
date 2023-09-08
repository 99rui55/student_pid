package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveAutoTrapezoid;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
  Chassis chassis = new Chassis();

  public RobotContainer() {
    SmartDashboard.putData("Trapezoid Command", new DriveAutoTrapezoid(chassis));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
