package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.VelocityPIDCommand;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Chassis chassis;
  VelocityPIDCommand pid;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    chassis = new Chassis();
    configureBindings();
    pid = new VelocityPIDCommand(chassis);
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
