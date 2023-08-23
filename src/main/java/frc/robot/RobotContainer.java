package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.VelocityPIDCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.commands.pcomcose;
import frc.robot.commands.pcomopen;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Chassis chassis;
  public final Climber climber;
  VelocityPIDCommand pid;
  pcomopen pcomopen;
  pcomcose pcomcose;
  CommandXboxController xbox;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    chassis = new Chassis();
    climber = new Climber();
    configureBindings();
    pid = new VelocityPIDCommand(chassis);
    pcomopen = new pcomopen(climber);
    pcomcose = new pcomcose(climber);
    xbox = new CommandXboxController(0);
  }

  private void configureBindings() {
    xbox.x().onTrue(pcomcose);
    xbox.b().onTrue(pcomcose);
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
