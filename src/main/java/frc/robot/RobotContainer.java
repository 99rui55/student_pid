package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.VelocityPIDCommand;
import frc.robot.commands.udi;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Chassis chassis;
  VelocityPIDCommand pid;
  udi udi;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    chassis = new Chassis();
    configureBindings();
    pid = new VelocityPIDCommand(chassis);
    udi = new udi(chassis, 2, 0.2, 0.05);
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return udi;
  }
}